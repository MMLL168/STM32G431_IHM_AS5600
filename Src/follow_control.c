#include "follow_control.h"

#include "as5600_follower.h"
#include "as5600_knob.h"
#include "drive_parameters.h"
#include "main.h"
#include "mc_api.h"
#include "parameters_conversion.h"
#include "user_sensored.h"

#define FOLLOW_CONTROL_TASK_PERIOD_MS 10U
#define FOLLOW_CONTROL_HOLD_ENTER_DEG10 10
#define FOLLOW_CONTROL_HOLD_EXIT_DEG10 20
#define FOLLOW_CONTROL_SPEED_ZERO_WINDOW_RPM 40
#define FOLLOW_CONTROL_SPEED_KP_RPM_PER_DEG 3
#define FOLLOW_CONTROL_SPEED_KI_DIVISOR 20000
#define FOLLOW_CONTROL_SPEED_KI_ENABLE 0
#define FOLLOW_CONTROL_SPEED_KD_RPM_PER_RPM 0
#define FOLLOW_CONTROL_SPEED_TO_CURRENT_KP_MA_PER_RPM 1
#define FOLLOW_CONTROL_TARGET_ANGLE_SLEW_MIN_DEG10_PER_TICK 8
#define FOLLOW_CONTROL_TARGET_ANGLE_SLEW_MAX_DEG10_PER_TICK 80
#define FOLLOW_CONTROL_TARGET_ANGLE_SLEW_DIVISOR 6
#define FOLLOW_CONTROL_MAX_RPM 240
#define FOLLOW_CONTROL_MIN_RPM 0
#define FOLLOW_CONTROL_MAX_CURRENT_MA 180
#define FOLLOW_CONTROL_CURRENT_ZERO_WINDOW_MA 20
#define FOLLOW_CONTROL_CURRENT_SLEW_MA_PER_TICK 24
#define FOLLOW_CONTROL_SOURCE_DIRECTION_SIGN 1
#define FOLLOW_CONTROL_FOLLOWER_DIRECTION_SIGN 1
#define FOLLOW_CONTROL_DIRECTION_PROBE_ENABLE 0
#define FOLLOW_CONTROL_DIRECTION_PROBE_TARGET_ABS_RPM 400
#define FOLLOW_CONTROL_DIRECTION_PROBE_MAX_CURRENT_MA 300
#define FOLLOW_CONTROL_DIRECTION_PROBE_SPEED_TO_CURRENT_KP_MA_PER_RPM 1
#define FOLLOW_CONTROL_DIRECTION_PROBE_OUTPUT_SIGN 1
#define FOLLOW_CONTROL_DIRECTION_PROBE_PHASE_DURATION_MS 1500U
#define FOLLOW_CONTROL_DIRECTION_PROBE_SWEEP_ENABLE 0
#define FOLLOW_CONTROL_DIRECTION_PROBE_FIXED_PHASE_INDEX 6U

typedef struct
{
  int8_t direction_sign;
  int16_t electrical_trim_deg10;
} FollowControlProbePhase_t;

volatile FollowControlSnapshot_t g_follow_control = {0};

static const FollowControlProbePhase_t s_follow_probe_phase_table[] = {
    {1, 0},
    {1, 900},
    {1, 1800},
    {1, -900},
    {-1, 0},
    {-1, 900},
    {-1, 1800},
    {-1, -900},
};

#define FOLLOW_CONTROL_DIRECTION_PROBE_PHASE_COUNT \
  ((uint8_t)(sizeof(s_follow_probe_phase_table) / sizeof(s_follow_probe_phase_table[0])))

static int32_t s_follow_integral_accum = 0;
static int32_t s_follow_target_rpm_cmd = 0;
static int32_t s_follow_current_cmd_ma = 0;
static int32_t s_follow_filtered_speed_rpm = 0;
static uint8_t s_follow_hold_active = 1U;
static uint8_t s_follow_alignment_valid = 0U;
static uint32_t s_follow_next_tick_ms = 0U;
static uint32_t s_follow_probe_next_switch_ms = 0U;
static uint16_t s_follow_command_angle_deg10 = 0U;
static int16_t s_follow_angle_offset_deg10 = 0;
static uint8_t s_follow_probe_phase_index = 0U;

static uint16_t FollowControl_NormalizeAngleDeg10(int32_t angle_deg10)
{
  while (angle_deg10 < 0)
  {
    angle_deg10 += 3600;
  }

  while (angle_deg10 >= 3600)
  {
    angle_deg10 -= 3600;
  }

  return (uint16_t)angle_deg10;
}

static int16_t FollowControl_WrapAngleErrorDeg10(uint16_t target_angle_deg10,
                                                 uint16_t actual_angle_deg10)
{
  int32_t error = (int32_t)target_angle_deg10 - (int32_t)actual_angle_deg10;

  if (error > 1800)
  {
    error -= 3600;
  }
  else if (error < -1800)
  {
    error += 3600;
  }

  return (int16_t)error;
}

static int32_t FollowControl_AbsI32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static int32_t FollowControl_ClampI32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

static uint16_t FollowControl_ApplyDirection(uint16_t angle_deg10, int32_t direction_sign)
{
  const int32_t signed_angle_deg10 = (int32_t)angle_deg10 * direction_sign;
  return FollowControl_NormalizeAngleDeg10(signed_angle_deg10);
}

static void FollowControl_ResetDynamicState(void)
{
  s_follow_integral_accum = 0;
  s_follow_target_rpm_cmd = 0;
  s_follow_current_cmd_ma = 0;
  s_follow_filtered_speed_rpm = 0;
  s_follow_hold_active = 1U;
  s_follow_alignment_valid = 0U;
  s_follow_angle_offset_deg10 = 0;
  s_follow_command_angle_deg10 = 0U;
  s_follow_probe_phase_index = 0U;
  s_follow_probe_next_switch_ms = 0U;
  UserSensored_SetRuntimeDirectionSign(1);
  UserSensored_SetRuntimeElectricalTrimDeg10(0);
}

static void FollowControl_StopMotorIfRunning(void)
{
  const MCI_State_t state = MC_GetSTMStateMotor1();

  if ((state == RUN) || (state == START) || (state == CHARGE_BOOT_CAP) ||
      (state == OFFSET_CALIB) || (state == SWITCH_OVER) ||
      (state == ALIGNMENT) || (state == STOP))
  {
    (void)MC_StopMotor1();
  }
}

static int16_t FollowControl_CurrentMilliAmpToDigit(int32_t current_milli_amp)
{
  return (int16_t)((current_milli_amp * CURRENT_CONV_FACTOR) / 1000);
}

static void FollowControl_StartOrUpdateCurrentCommand(int32_t target_current_ma)
{
  const MCI_State_t state = MC_GetSTMStateMotor1();
  qd_t iqd_ref = {0};

  iqd_ref.q = FollowControl_CurrentMilliAmpToDigit(target_current_ma);
  iqd_ref.d = 0;

  switch (state)
  {
    case IDLE:
      MC_SetCurrentReferenceMotor1(iqd_ref);
      (void)MC_StartMotor1();
      break;

    case RUN:
    case START:
    case CHARGE_BOOT_CAP:
    case OFFSET_CALIB:
    case SWITCH_OVER:
    case ALIGNMENT:
      MC_SetCurrentReferenceMotor1(iqd_ref);
      break;

    case FAULT_OVER:
      (void)MC_AcknowledgeFaultMotor1();
      break;

    default:
      break;
  }
}

static void FollowControl_CommandZeroCurrent(void)
{
  const MCI_State_t state = MC_GetSTMStateMotor1();

  if ((state == RUN) || (state == START) || (state == CHARGE_BOOT_CAP) ||
      (state == OFFSET_CALIB) || (state == SWITCH_OVER) ||
      (state == ALIGNMENT))
  {
    FollowControl_StartOrUpdateCurrentCommand(0);
  }
}

void FollowControl_Init(void)
{
  g_follow_control.target_angle_deg10 = 0U;
  g_follow_control.actual_angle_deg10 = 0U;
  g_follow_control.angle_error_deg10 = 0;
  g_follow_control.target_rpm = 0;
  g_follow_control.filtered_speed_rpm = 0;
  g_follow_control.integral_rpm = 0;
  g_follow_control.active = 0U;
  g_follow_control.hold_active = 1U;
  g_follow_control.source_online = 0U;
  g_follow_control.follower_online = 0U;
  g_follow_control.reserved0 = 0U;
  s_follow_next_tick_ms = HAL_GetTick();
  FollowControl_ResetDynamicState();
}

void FollowControl_Task(void)
{
  const uint32_t now_ms = HAL_GetTick();
  int16_t angle_error_deg10;
  int32_t abs_error_deg10;
  int32_t raw_target_rpm;
  int32_t target_rpm;
  int32_t measured_speed_rpm;
  int32_t filtered_speed_rpm;
  int32_t integral_rpm;
  int32_t integral_limit;
  int32_t probe_abs_speed_rpm;
  int32_t probe_current_ma;
  int32_t probe_speed_deficit_rpm;
  int32_t command_target_step_deg10;
  int16_t command_target_error_deg10;
  int32_t speed_error_rpm;
  int32_t target_current_ma;
  uint16_t command_target_angle_deg10;
  uint16_t mapped_target_angle_deg10;
  uint16_t mapped_follower_angle_deg10;
  uint16_t aligned_follower_angle_deg10;

  if (now_ms < s_follow_next_tick_ms)
  {
    return;
  }

  s_follow_next_tick_ms = now_ms + FOLLOW_CONTROL_TASK_PERIOD_MS;

  g_follow_control.source_online = g_as5600_knob.online;
  g_follow_control.follower_online = g_as5600_follower.online;

  if ((g_as5600_knob.online == 0U) || (g_as5600_follower.online == 0U))
  {
    FollowControl_ResetDynamicState();
    g_follow_control.angle_error_deg10 = 0;
    g_follow_control.target_rpm = 0;
    g_follow_control.filtered_speed_rpm = 0;
    g_follow_control.integral_rpm = 0;
    g_follow_control.active = 0U;
    g_follow_control.hold_active = 1U;
    g_follow_control.reserved0 = 0U;
    FollowControl_StopMotorIfRunning();
    return;
  }

  mapped_target_angle_deg10 =
      FollowControl_ApplyDirection(g_as5600_knob.angle_deg10, FOLLOW_CONTROL_SOURCE_DIRECTION_SIGN);
  mapped_follower_angle_deg10 =
      FollowControl_ApplyDirection(g_as5600_follower.angle_deg10, FOLLOW_CONTROL_FOLLOWER_DIRECTION_SIGN);

  if (FOLLOW_CONTROL_DIRECTION_PROBE_ENABLE != 0)
  {
    const FollowControlProbePhase_t *probe_phase;

    if (FOLLOW_CONTROL_DIRECTION_PROBE_SWEEP_ENABLE == 0)
    {
      s_follow_probe_phase_index = FOLLOW_CONTROL_DIRECTION_PROBE_FIXED_PHASE_INDEX;
    }
    else if (s_follow_probe_next_switch_ms == 0U)
    {
      s_follow_probe_phase_index = 0U;
      s_follow_probe_next_switch_ms = now_ms + FOLLOW_CONTROL_DIRECTION_PROBE_PHASE_DURATION_MS;
    }
    else
    {
      while ((int32_t)(now_ms - s_follow_probe_next_switch_ms) >= 0)
      {
        s_follow_probe_phase_index =
            (uint8_t)((s_follow_probe_phase_index + 1U) % FOLLOW_CONTROL_DIRECTION_PROBE_PHASE_COUNT);
        s_follow_probe_next_switch_ms += FOLLOW_CONTROL_DIRECTION_PROBE_PHASE_DURATION_MS;
      }
    }

    probe_phase = &s_follow_probe_phase_table[s_follow_probe_phase_index];
    UserSensored_SetRuntimeDirectionSign(probe_phase->direction_sign);
    UserSensored_SetRuntimeElectricalTrimDeg10(probe_phase->electrical_trim_deg10);
    measured_speed_rpm = (int32_t)UserSensored_GetMechanicalSpeedRpm();
    probe_abs_speed_rpm = FollowControl_AbsI32(measured_speed_rpm);
    probe_speed_deficit_rpm = FOLLOW_CONTROL_DIRECTION_PROBE_TARGET_ABS_RPM - probe_abs_speed_rpm;

    if (probe_speed_deficit_rpm > 0)
    {
      probe_current_ma = probe_speed_deficit_rpm * FOLLOW_CONTROL_DIRECTION_PROBE_SPEED_TO_CURRENT_KP_MA_PER_RPM;
      probe_current_ma = FollowControl_ClampI32(probe_current_ma,
                                                0,
                                                FOLLOW_CONTROL_DIRECTION_PROBE_MAX_CURRENT_MA);
      probe_current_ma *= FOLLOW_CONTROL_DIRECTION_PROBE_OUTPUT_SIGN;
    }
    else
    {
      probe_current_ma = 0;
    }

    g_follow_control.target_angle_deg10 = mapped_target_angle_deg10;
    g_follow_control.actual_angle_deg10 = mapped_follower_angle_deg10;
    g_follow_control.angle_error_deg10 = 0;
    g_follow_control.target_rpm =
      FOLLOW_CONTROL_DIRECTION_PROBE_OUTPUT_SIGN * FOLLOW_CONTROL_DIRECTION_PROBE_TARGET_ABS_RPM;
    g_follow_control.filtered_speed_rpm = (int16_t)measured_speed_rpm;
    g_follow_control.integral_rpm = 0;
    g_follow_control.active = 1U;
    g_follow_control.hold_active = 0U;
    g_follow_control.reserved0 = s_follow_probe_phase_index;
    FollowControl_StartOrUpdateCurrentCommand(probe_current_ma);
    return;
  }

  if (s_follow_alignment_valid == 0U)
  {
    s_follow_angle_offset_deg10 =
        FollowControl_WrapAngleErrorDeg10(mapped_target_angle_deg10, mapped_follower_angle_deg10);
    s_follow_command_angle_deg10 = mapped_target_angle_deg10;
    s_follow_alignment_valid = 1U;
  }

  command_target_angle_deg10 = s_follow_command_angle_deg10;
  command_target_error_deg10 = FollowControl_WrapAngleErrorDeg10(mapped_target_angle_deg10,
                                                                 command_target_angle_deg10);
  command_target_step_deg10 = FollowControl_AbsI32((int32_t)command_target_error_deg10) /
                              FOLLOW_CONTROL_TARGET_ANGLE_SLEW_DIVISOR;
  command_target_step_deg10 = FollowControl_ClampI32(command_target_step_deg10,
                                                     FOLLOW_CONTROL_TARGET_ANGLE_SLEW_MIN_DEG10_PER_TICK,
                                                     FOLLOW_CONTROL_TARGET_ANGLE_SLEW_MAX_DEG10_PER_TICK);

  if (command_target_error_deg10 > command_target_step_deg10)
  {
    command_target_angle_deg10 = FollowControl_NormalizeAngleDeg10((int32_t)command_target_angle_deg10 +
                                                                   command_target_step_deg10);
  }
  else if (command_target_error_deg10 < -command_target_step_deg10)
  {
    command_target_angle_deg10 = FollowControl_NormalizeAngleDeg10((int32_t)command_target_angle_deg10 -
                                                                   command_target_step_deg10);
  }
  else
  {
    command_target_angle_deg10 = mapped_target_angle_deg10;
  }

  s_follow_command_angle_deg10 = command_target_angle_deg10;

  aligned_follower_angle_deg10 =
      FollowControl_NormalizeAngleDeg10((int32_t)mapped_follower_angle_deg10 + (int32_t)s_follow_angle_offset_deg10);

  g_follow_control.target_angle_deg10 = command_target_angle_deg10;
  g_follow_control.actual_angle_deg10 = aligned_follower_angle_deg10;

  angle_error_deg10 = FollowControl_WrapAngleErrorDeg10(command_target_angle_deg10,
                                                        aligned_follower_angle_deg10);
  abs_error_deg10 = FollowControl_AbsI32(angle_error_deg10);

  g_follow_control.angle_error_deg10 = angle_error_deg10;

  measured_speed_rpm = (int32_t)UserSensored_GetMechanicalSpeedRpm();
  s_follow_filtered_speed_rpm += (measured_speed_rpm - s_follow_filtered_speed_rpm) >> 1;
  filtered_speed_rpm = s_follow_filtered_speed_rpm;

  if (s_follow_hold_active != 0U)
  {
    if (abs_error_deg10 >= FOLLOW_CONTROL_HOLD_EXIT_DEG10)
    {
      s_follow_hold_active = 0U;
      s_follow_target_rpm_cmd = (angle_error_deg10 >= 0)
                                    ? (int32_t)FOLLOW_CONTROL_MIN_RPM
                                    : -(int32_t)FOLLOW_CONTROL_MIN_RPM;
    }
  }
  else if ((abs_error_deg10 <= FOLLOW_CONTROL_HOLD_ENTER_DEG10) &&
           (FollowControl_AbsI32(filtered_speed_rpm) <= FOLLOW_CONTROL_SPEED_ZERO_WINDOW_RPM))
  {
    s_follow_hold_active = 1U;
  }

  if (s_follow_hold_active != 0U)
  {
    s_follow_integral_accum = 0;
    s_follow_target_rpm_cmd = 0;
    s_follow_current_cmd_ma = 0;
    g_follow_control.target_rpm = 0;
    g_follow_control.filtered_speed_rpm = (int16_t)filtered_speed_rpm;
    g_follow_control.integral_rpm = 0;
    g_follow_control.active = 0U;
    g_follow_control.hold_active = 1U;
    FollowControl_CommandZeroCurrent();
    return;
  }

  if (FOLLOW_CONTROL_SPEED_KI_ENABLE != 0)
  {
    integral_limit = FOLLOW_CONTROL_MAX_RPM * FOLLOW_CONTROL_SPEED_KI_DIVISOR;
    s_follow_integral_accum += (int32_t)angle_error_deg10 * (int32_t)FOLLOW_CONTROL_TASK_PERIOD_MS;
    s_follow_integral_accum = FollowControl_ClampI32(s_follow_integral_accum, -integral_limit, integral_limit);
    integral_rpm = s_follow_integral_accum / FOLLOW_CONTROL_SPEED_KI_DIVISOR;
  }
  else
  {
    s_follow_integral_accum = 0;
    integral_rpm = 0;
  }

  raw_target_rpm = ((int32_t)angle_error_deg10 * FOLLOW_CONTROL_SPEED_KP_RPM_PER_DEG) / 10;
  raw_target_rpm += integral_rpm;
  raw_target_rpm -= filtered_speed_rpm * FOLLOW_CONTROL_SPEED_KD_RPM_PER_RPM;
  raw_target_rpm = FollowControl_ClampI32(raw_target_rpm,
                                          -FOLLOW_CONTROL_MAX_RPM,
                                          FOLLOW_CONTROL_MAX_RPM);

  if ((raw_target_rpm > 0) && (raw_target_rpm < FOLLOW_CONTROL_MIN_RPM))
  {
    raw_target_rpm = FOLLOW_CONTROL_MIN_RPM;
  }
  else if ((raw_target_rpm < 0) && (raw_target_rpm > -FOLLOW_CONTROL_MIN_RPM))
  {
    raw_target_rpm = -FOLLOW_CONTROL_MIN_RPM;
  }

  s_follow_target_rpm_cmd = raw_target_rpm;

  target_rpm = FollowControl_ClampI32(s_follow_target_rpm_cmd,
                                      -MAX_APPLICATION_SPEED_RPM,
                                      MAX_APPLICATION_SPEED_RPM);

  speed_error_rpm = target_rpm - filtered_speed_rpm;
  target_current_ma = speed_error_rpm * FOLLOW_CONTROL_SPEED_TO_CURRENT_KP_MA_PER_RPM;
  target_current_ma = FollowControl_ClampI32(target_current_ma,
                                             -FOLLOW_CONTROL_MAX_CURRENT_MA,
                                             FOLLOW_CONTROL_MAX_CURRENT_MA);

  if (((target_current_ma > 0) && (s_follow_current_cmd_ma < 0)) ||
      ((target_current_ma < 0) && (s_follow_current_cmd_ma > 0)) ||
      (target_current_ma == 0))
  {
    s_follow_current_cmd_ma = target_current_ma;
  }
  else if (target_current_ma > s_follow_current_cmd_ma)
  {
    s_follow_current_cmd_ma += FOLLOW_CONTROL_CURRENT_SLEW_MA_PER_TICK;
    if (s_follow_current_cmd_ma > target_current_ma)
    {
      s_follow_current_cmd_ma = target_current_ma;
    }
  }
  else if (target_current_ma < s_follow_current_cmd_ma)
  {
    s_follow_current_cmd_ma -= FOLLOW_CONTROL_CURRENT_SLEW_MA_PER_TICK;
    if (s_follow_current_cmd_ma < target_current_ma)
    {
      s_follow_current_cmd_ma = target_current_ma;
    }
  }

  if ((FollowControl_AbsI32(s_follow_current_cmd_ma) <= FOLLOW_CONTROL_CURRENT_ZERO_WINDOW_MA) &&
      (s_follow_hold_active != 0U))
  {
    s_follow_current_cmd_ma = 0;
  }

  g_follow_control.target_rpm = (int16_t)target_rpm;
  g_follow_control.filtered_speed_rpm = (int16_t)filtered_speed_rpm;
  g_follow_control.integral_rpm = (int16_t)integral_rpm;
  g_follow_control.active = 1U;
  g_follow_control.hold_active = 0U;

  FollowControl_StartOrUpdateCurrentCommand(s_follow_current_cmd_ma);
}