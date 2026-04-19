#include "follow_control.h"

#include "follow_tuning_runtime.h"
#include "tuning_config.h"
#include "as5600_follower.h"
#include "as5600_knob.h"
#include "drive_parameters.h"
#include "main.h"
#include "mc_api.h"
#include "parameters_conversion.h"
#include "user_sensored.h"

#define FOLLOW_CONTROL_TASK_PERIOD_MS 10U
#define FOLLOW_CONTROL_CURRENT_SLEW_MA_PER_TICK 10
#define FOLLOW_CONTROL_CURRENT_ZERO_CROSS_SLEW_MA_PER_TICK 20
#define FOLLOW_CONTROL_STATUS_MANUAL_ALIGN_HOLD 3U
#define FC_CFG(field) (FollowTuning_GetProfile()->field)

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
static int32_t s_follow_command_step_deg10 = 0;
static uint16_t s_follow_last_actual_angle_deg10 = 0U;
static uint16_t s_follow_outer_actual_angle_deg10 = 0U;
static uint16_t s_follow_test_center_angle_deg10 = 0U;
static uint8_t s_follow_hold_active = 1U;
static uint8_t s_follow_alignment_valid = 0U;
static uint8_t s_follow_has_speed_sample = 0U;
static uint8_t s_follow_test_center_valid = 0U;
static uint8_t s_follow_test_step_positive = 0U;
static uint32_t s_follow_next_tick_ms = 0U;
static uint32_t s_follow_probe_next_switch_ms = 0U;
static uint32_t s_follow_test_next_switch_ms = 0U;
static uint16_t s_follow_command_angle_deg10 = 0U;
static uint16_t s_follow_manual_hold_target_angle_deg10 = 0U;
static int16_t s_follow_angle_offset_deg10 = 0;
static int16_t s_follow_source_align_offset_deg10 = 0;
static uint8_t s_follow_probe_phase_index = 0U;
static uint8_t s_follow_manual_align_hold_active = 0U;

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

static int32_t FollowControl_SlewTowardI32(int32_t current_value, int32_t target_value, int32_t step)
{
  if (target_value > current_value)
  {
    current_value += step;
    if (current_value > target_value)
    {
      current_value = target_value;
    }
  }
  else if (target_value < current_value)
  {
    current_value -= step;
    if (current_value < target_value)
    {
      current_value = target_value;
    }
  }

  return current_value;
}

static int32_t FollowControl_SlewCurrentCommandI32(int32_t current_value, int32_t target_value)
{
  if (((current_value > 0) && (target_value < 0)) ||
      ((current_value < 0) && (target_value > 0)))
  {
    return FollowControl_SlewTowardI32(current_value,
                                       0,
                                       FOLLOW_CONTROL_CURRENT_ZERO_CROSS_SLEW_MA_PER_TICK);
  }

  return FollowControl_SlewTowardI32(current_value,
                                     target_value,
                                     FOLLOW_CONTROL_CURRENT_SLEW_MA_PER_TICK);
}

static uint16_t FollowControl_FilterAngleDeg10(uint16_t current_angle_deg10, uint16_t target_angle_deg10)
{
  const int32_t filter_error_deg10 =
      (int32_t)FollowControl_WrapAngleErrorDeg10(target_angle_deg10, current_angle_deg10);
  int32_t filter_step_deg10 = filter_error_deg10 >> FC_CFG(outer_angle_filter_shift);

  if ((filter_step_deg10 == 0) && (filter_error_deg10 != 0))
  {
    filter_step_deg10 = (filter_error_deg10 > 0) ? 1 : -1;
  }

  return FollowControl_NormalizeAngleDeg10((int32_t)current_angle_deg10 + filter_step_deg10);
}

static uint16_t FollowControl_ApplyDirection(uint16_t angle_deg10, int32_t direction_sign)
{
  const int32_t signed_angle_deg10 = (int32_t)angle_deg10 * direction_sign;
  return FollowControl_NormalizeAngleDeg10(signed_angle_deg10);
}

static uint16_t FollowControl_ApplySourceAlignmentOffset(uint16_t target_angle_deg10)
{
  return FollowControl_NormalizeAngleDeg10((int32_t)target_angle_deg10 +
                                           (int32_t)s_follow_source_align_offset_deg10);
}

static uint16_t FollowControl_GetSelectedTargetAngleDeg10(uint32_t now_ms,
                                                          uint16_t knob_target_angle_deg10,
                                                          uint16_t follower_angle_deg10)
{
  if (FC_CFG(test_mode) == FOLLOW_CONTROL_TEST_MODE_NORMAL_FOLLOW)
  {
    return knob_target_angle_deg10;
  }

  if (s_follow_test_center_valid == 0U)
  {
    s_follow_test_center_angle_deg10 = follower_angle_deg10;
    s_follow_test_center_valid = 1U;
    s_follow_test_step_positive = (FC_CFG(step_start_positive) != 0) ? 1U : 0U;
    s_follow_test_next_switch_ms = now_ms + (uint32_t)FC_CFG(step_dwell_ms);
  }

  if (FC_CFG(test_mode) == FOLLOW_CONTROL_TEST_MODE_FIXED_TARGET)
  {
    return FollowControl_NormalizeAngleDeg10((int32_t)s_follow_test_center_angle_deg10 +
                                             FC_CFG(fixed_target_offset_deg10));
  }

  if (FC_CFG(test_mode) == FOLLOW_CONTROL_TEST_MODE_STEP_SEQUENCE)
  {
    while ((int32_t)(now_ms - s_follow_test_next_switch_ms) >= 0)
    {
      s_follow_test_step_positive = (s_follow_test_step_positive == 0U) ? 1U : 0U;
      s_follow_test_next_switch_ms += (uint32_t)FC_CFG(step_dwell_ms);
    }

    return FollowControl_NormalizeAngleDeg10((int32_t)s_follow_test_center_angle_deg10 +
                                             ((s_follow_test_step_positive != 0U)
                                                  ? FC_CFG(step_amplitude_deg10)
                                                  : -FC_CFG(step_amplitude_deg10)));
  }

  return knob_target_angle_deg10;
}

static void FollowControl_ResetDynamicState(void)
{
  s_follow_integral_accum = 0;
  s_follow_target_rpm_cmd = 0;
  s_follow_current_cmd_ma = 0;
  s_follow_filtered_speed_rpm = 0;
  s_follow_command_step_deg10 = 0;
  s_follow_last_actual_angle_deg10 = 0U;
  s_follow_outer_actual_angle_deg10 = 0U;
  s_follow_test_center_angle_deg10 = 0U;
  s_follow_manual_hold_target_angle_deg10 = 0U;
  s_follow_hold_active = 1U;
  s_follow_alignment_valid = 0U;
  s_follow_has_speed_sample = 0U;
  s_follow_test_center_valid = 0U;
  s_follow_test_step_positive = 0U;
  s_follow_angle_offset_deg10 = 0;
  s_follow_command_angle_deg10 = 0U;
  s_follow_probe_phase_index = 0U;
  s_follow_manual_align_hold_active = 0U;
  s_follow_probe_next_switch_ms = 0U;
  s_follow_test_next_switch_ms = 0U;
  UserSensored_SetRuntimeDirectionSign(1);
  UserSensored_SetRuntimeElectricalTrimDeg10(0);
}

static void FollowControl_StopMotorIfRunning(void)
{
  const MCI_State_t state = MC_GetSTMStateMotor1();

  if ((state == RUN) || (state == START) || (state == CHARGE_BOOT_CAP) ||
      (state == OFFSET_CALIB) || (state == SWITCH_OVER) ||
      (state == ALIGNMENT))
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
  FollowTuning_Init();
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
  s_follow_source_align_offset_deg10 = 0;
  s_follow_next_tick_ms = HAL_GetTick();
  FollowControl_ResetDynamicState();
}

void FollowControl_OnTuningProfileChanged(void)
{
  s_follow_source_align_offset_deg10 = 0;
  FollowControl_ResetDynamicState();
  g_follow_control.angle_error_deg10 = 0;
  g_follow_control.target_rpm = 0;
  g_follow_control.filtered_speed_rpm = 0;
  g_follow_control.integral_rpm = 0;
  g_follow_control.active = 0U;
  g_follow_control.hold_active = 1U;
  g_follow_control.reserved0 = 0U;
  s_follow_next_tick_ms = HAL_GetTick();
  FollowControl_CommandZeroCurrent();
}

uint8_t FollowControl_BeginManualAlignHold(void)
{
  uint16_t mapped_follower_angle_deg10;

  if ((FC_CFG(test_mode) != FOLLOW_CONTROL_TEST_MODE_NORMAL_FOLLOW) ||
      (g_as5600_follower.online == 0U))
  {
    return 0U;
  }

  mapped_follower_angle_deg10 =
      FollowControl_ApplyDirection(g_as5600_follower.angle_deg10, FC_CFG(follower_direction_sign));

  s_follow_manual_hold_target_angle_deg10 = mapped_follower_angle_deg10;
  s_follow_manual_align_hold_active = 1U;
  s_follow_alignment_valid = 1U;
  s_follow_hold_active = 1U;
  s_follow_integral_accum = 0;
  s_follow_target_rpm_cmd = 0;
  s_follow_current_cmd_ma = 0;
  s_follow_filtered_speed_rpm = 0;
  s_follow_command_step_deg10 = 0;
  s_follow_command_angle_deg10 = mapped_follower_angle_deg10;
  s_follow_outer_actual_angle_deg10 = mapped_follower_angle_deg10;
  s_follow_last_actual_angle_deg10 = mapped_follower_angle_deg10;
  s_follow_has_speed_sample = 0U;

  return 1U;
}

void FollowControl_EndManualAlignHold(void)
{
  uint16_t mapped_target_angle_deg10;
  uint16_t mapped_follower_angle_deg10;

  if (s_follow_manual_align_hold_active == 0U)
  {
    return;
  }

  if ((FC_CFG(test_mode) == FOLLOW_CONTROL_TEST_MODE_NORMAL_FOLLOW) &&
      (g_as5600_knob.online != 0U) &&
      (g_as5600_follower.online != 0U))
  {
    mapped_target_angle_deg10 =
        FollowControl_ApplyDirection(g_as5600_knob.angle_deg10, FC_CFG(source_direction_sign));
    mapped_follower_angle_deg10 =
        FollowControl_ApplyDirection(g_as5600_follower.angle_deg10, FC_CFG(follower_direction_sign));

    s_follow_source_align_offset_deg10 =
        FollowControl_WrapAngleErrorDeg10(mapped_follower_angle_deg10, mapped_target_angle_deg10);
    s_follow_command_angle_deg10 = mapped_follower_angle_deg10;
    s_follow_outer_actual_angle_deg10 = mapped_follower_angle_deg10;
    s_follow_last_actual_angle_deg10 = mapped_follower_angle_deg10;
    s_follow_integral_accum = 0;
    s_follow_target_rpm_cmd = 0;
    s_follow_current_cmd_ma = 0;
    s_follow_filtered_speed_rpm = 0;
    s_follow_command_step_deg10 = 0;
    s_follow_has_speed_sample = 0U;
    s_follow_hold_active = 1U;
  }

  s_follow_manual_align_hold_active = 0U;
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
  int32_t abs_filtered_speed_rpm;
  int32_t integral_rpm;
  int32_t integral_limit;
  int32_t probe_abs_speed_rpm;
  int32_t probe_current_ma;
  int32_t probe_speed_deficit_rpm;
  int32_t desired_command_step_deg10;
  int32_t command_target_step_deg10;
  int16_t command_target_error_deg10;
  int32_t max_target_rpm_limit;
  int32_t max_current_limit_ma;
  int32_t position_current_ma;
  int32_t abs_position_current_ma;
  int32_t damping_current_ma;
  int32_t target_current_ma;
  int32_t breakaway_floor_ma;
  int32_t breakaway_full_error_deg10;
  int32_t abs_target_rpm;
  int32_t motion_alignment;
  uint16_t command_target_angle_deg10;
  uint16_t mapped_target_angle_deg10;
  uint16_t mapped_follower_angle_deg10;
  uint16_t aligned_follower_angle_deg10;
  uint16_t outer_actual_angle_deg10;
  int16_t outer_filter_error_deg10;
  int32_t abs_outer_filter_error_deg10;
  uint8_t urgent_brake;

  if (now_ms < s_follow_next_tick_ms)
  {
    return;
  }

  s_follow_next_tick_ms = now_ms + FOLLOW_CONTROL_TASK_PERIOD_MS;

  g_follow_control.source_online =
      (FC_CFG(test_mode) == FOLLOW_CONTROL_TEST_MODE_NORMAL_FOLLOW) ? g_as5600_knob.online : 1U;
  g_follow_control.follower_online = g_as5600_follower.online;

    if (((FC_CFG(test_mode) == FOLLOW_CONTROL_TEST_MODE_NORMAL_FOLLOW) &&
       (g_as5600_knob.online == 0U)) ||
      (g_as5600_follower.online == 0U))
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
      FollowControl_ApplyDirection(g_as5600_knob.angle_deg10, FC_CFG(source_direction_sign));
  if (FC_CFG(test_mode) == FOLLOW_CONTROL_TEST_MODE_NORMAL_FOLLOW)
  {
    mapped_target_angle_deg10 = FollowControl_ApplySourceAlignmentOffset(mapped_target_angle_deg10);
  }
  mapped_follower_angle_deg10 =
      FollowControl_ApplyDirection(g_as5600_follower.angle_deg10, FC_CFG(follower_direction_sign));

  if ((FC_CFG(test_mode) == FOLLOW_CONTROL_TEST_MODE_NORMAL_FOLLOW) &&
      (s_follow_manual_align_hold_active != 0U))
  {
    mapped_target_angle_deg10 = s_follow_manual_hold_target_angle_deg10;
  }
  else
  {
    mapped_target_angle_deg10 =
        FollowControl_GetSelectedTargetAngleDeg10(now_ms, mapped_target_angle_deg10, mapped_follower_angle_deg10);
  }

  if (FC_CFG(test_mode) == FOLLOW_CONTROL_TEST_MODE_DIRECTION_PROBE)
  {
    const FollowControlProbePhase_t *probe_phase;

    if (FC_CFG(direction_probe_sweep_enable) == 0)
    {
      s_follow_probe_phase_index = (uint8_t)FC_CFG(direction_probe_fixed_phase_index);
    }
    else if (s_follow_probe_next_switch_ms == 0U)
    {
      s_follow_probe_phase_index = 0U;
      s_follow_probe_next_switch_ms = now_ms + (uint32_t)FC_CFG(direction_probe_phase_duration_ms);
    }
    else
    {
      while ((int32_t)(now_ms - s_follow_probe_next_switch_ms) >= 0)
      {
        s_follow_probe_phase_index =
            (uint8_t)((s_follow_probe_phase_index + 1U) % FOLLOW_CONTROL_DIRECTION_PROBE_PHASE_COUNT);
        s_follow_probe_next_switch_ms += (uint32_t)FC_CFG(direction_probe_phase_duration_ms);
      }
    }

    probe_phase = &s_follow_probe_phase_table[s_follow_probe_phase_index];
    UserSensored_SetRuntimeDirectionSign(probe_phase->direction_sign);
    UserSensored_SetRuntimeElectricalTrimDeg10(probe_phase->electrical_trim_deg10);
    measured_speed_rpm = (int32_t)UserSensored_GetMechanicalSpeedRpm();
    probe_abs_speed_rpm = FollowControl_AbsI32(measured_speed_rpm);
    probe_speed_deficit_rpm = FC_CFG(direction_probe_target_abs_rpm) - probe_abs_speed_rpm;

    if (probe_speed_deficit_rpm > 0)
    {
      probe_current_ma = probe_speed_deficit_rpm * FC_CFG(direction_probe_speed_to_current_kp_ma_per_rpm);
      probe_current_ma = FollowControl_ClampI32(probe_current_ma,
                                                0,
                                                FC_CFG(direction_probe_max_current_ma));
      probe_current_ma *= FC_CFG(direction_probe_output_sign);
    }
    else
    {
      probe_current_ma = 0;
    }

    g_follow_control.target_angle_deg10 = mapped_target_angle_deg10;
    g_follow_control.actual_angle_deg10 = mapped_follower_angle_deg10;
    g_follow_control.angle_error_deg10 = 0;
    g_follow_control.target_rpm =
      FC_CFG(direction_probe_output_sign) * FC_CFG(direction_probe_target_abs_rpm);
    g_follow_control.filtered_speed_rpm = (int16_t)measured_speed_rpm;
    g_follow_control.integral_rpm = 0;
    g_follow_control.active = 1U;
    g_follow_control.hold_active = 0U;
    g_follow_control.reserved0 = s_follow_probe_phase_index;
    s_follow_current_cmd_ma = probe_current_ma;
    FollowControl_StartOrUpdateCurrentCommand(probe_current_ma);
    return;
  }

  if (s_follow_alignment_valid == 0U)
  {
    s_follow_angle_offset_deg10 = 0;
    s_follow_command_angle_deg10 = mapped_follower_angle_deg10;
    s_follow_last_actual_angle_deg10 = mapped_follower_angle_deg10;
    s_follow_outer_actual_angle_deg10 = mapped_follower_angle_deg10;
    s_follow_has_speed_sample = 0U;
    s_follow_alignment_valid = 1U;
  }

  command_target_angle_deg10 = s_follow_command_angle_deg10;
  command_target_error_deg10 = FollowControl_WrapAngleErrorDeg10(mapped_target_angle_deg10,
                                                                 command_target_angle_deg10);
  desired_command_step_deg10 = FollowControl_AbsI32((int32_t)command_target_error_deg10) /
                               FC_CFG(target_angle_slew_divisor);
  if (desired_command_step_deg10 > 0)
  {
    desired_command_step_deg10 = FollowControl_ClampI32(desired_command_step_deg10,
                                                        FC_CFG(target_angle_slew_min_deg10_per_tick),
                                                        FC_CFG(target_angle_slew_max_deg10_per_tick));
  }

  s_follow_command_step_deg10 = FollowControl_SlewTowardI32(
      s_follow_command_step_deg10,
      desired_command_step_deg10,
      (desired_command_step_deg10 >= s_follow_command_step_deg10)
          ? FC_CFG(target_angle_slew_accel_deg10_per_tick)
          : FC_CFG(target_angle_slew_decel_deg10_per_tick));
  command_target_step_deg10 = s_follow_command_step_deg10;

  if ((command_target_step_deg10 > 0) && (command_target_error_deg10 > command_target_step_deg10))
  {
    command_target_angle_deg10 = FollowControl_NormalizeAngleDeg10((int32_t)command_target_angle_deg10 +
                                                                   command_target_step_deg10);
  }
  else if ((command_target_step_deg10 > 0) &&
           (command_target_error_deg10 < -command_target_step_deg10))
  {
    command_target_angle_deg10 = FollowControl_NormalizeAngleDeg10((int32_t)command_target_angle_deg10 -
                                                                   command_target_step_deg10);
  }
  else
  {
    command_target_angle_deg10 = mapped_target_angle_deg10;
    s_follow_command_step_deg10 = 0;
  }

  s_follow_command_angle_deg10 = command_target_angle_deg10;

  aligned_follower_angle_deg10 =
      FollowControl_NormalizeAngleDeg10((int32_t)mapped_follower_angle_deg10 + (int32_t)s_follow_angle_offset_deg10);
    outer_filter_error_deg10 =
      FollowControl_WrapAngleErrorDeg10(aligned_follower_angle_deg10, s_follow_outer_actual_angle_deg10);
    abs_outer_filter_error_deg10 = FollowControl_AbsI32((int32_t)outer_filter_error_deg10);

    if ((FC_CFG(outer_angle_filter_shift) == 0) ||
      (abs_outer_filter_error_deg10 >= FC_CFG(hold_exit_deg10)))
    {
    s_follow_outer_actual_angle_deg10 = aligned_follower_angle_deg10;
    }
    else
    {
    s_follow_outer_actual_angle_deg10 =
      FollowControl_FilterAngleDeg10(s_follow_outer_actual_angle_deg10, aligned_follower_angle_deg10);
    }

  outer_actual_angle_deg10 = s_follow_outer_actual_angle_deg10;

  g_follow_control.target_angle_deg10 = command_target_angle_deg10;
  g_follow_control.actual_angle_deg10 = outer_actual_angle_deg10;

  angle_error_deg10 = FollowControl_WrapAngleErrorDeg10(command_target_angle_deg10,
                                                        outer_actual_angle_deg10);
  abs_error_deg10 = FollowControl_AbsI32(angle_error_deg10);

  g_follow_control.angle_error_deg10 = angle_error_deg10;

  if (s_follow_has_speed_sample != 0U)
  {
    const int32_t delta_angle_deg10 =
      (int32_t)FollowControl_WrapAngleErrorDeg10(outer_actual_angle_deg10,
                                                   s_follow_last_actual_angle_deg10);

    measured_speed_rpm = (delta_angle_deg10 * 60000L) /
                         (3600L * (int32_t)FOLLOW_CONTROL_TASK_PERIOD_MS);
  }
  else
  {
    measured_speed_rpm = 0;
    s_follow_has_speed_sample = 1U;
  }

  s_follow_last_actual_angle_deg10 = outer_actual_angle_deg10;
  s_follow_filtered_speed_rpm += (measured_speed_rpm - s_follow_filtered_speed_rpm) >> 1;
  filtered_speed_rpm = s_follow_filtered_speed_rpm;
  abs_filtered_speed_rpm = FollowControl_AbsI32(filtered_speed_rpm);

  if (s_follow_hold_active != 0U)
  {
    if ((abs_error_deg10 >= FC_CFG(hold_exit_deg10)) ||
        ((abs_error_deg10 >= FC_CFG(hold_enter_deg10)) &&
         (abs_filtered_speed_rpm >= FC_CFG(speed_zero_window_rpm))))
    {
      s_follow_hold_active = 0U;
      s_follow_target_rpm_cmd = (angle_error_deg10 >= 0) ? FC_CFG(min_rpm) : -FC_CFG(min_rpm);
    }
  }
  else if ((abs_error_deg10 <= FC_CFG(hold_enter_deg10)) &&
           (abs_filtered_speed_rpm <= FC_CFG(speed_zero_window_rpm)))
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
    g_follow_control.reserved0 =
      (s_follow_manual_align_hold_active != 0U) ? FOLLOW_CONTROL_STATUS_MANUAL_ALIGN_HOLD : 0U;
    FollowControl_CommandZeroCurrent();
    return;
  }

  if (FC_CFG(speed_ki_enable) != 0)
  {
    integral_limit = FC_CFG(max_rpm) * FC_CFG(speed_ki_divisor);
    s_follow_integral_accum += (int32_t)angle_error_deg10 * (int32_t)FOLLOW_CONTROL_TASK_PERIOD_MS;
    s_follow_integral_accum = FollowControl_ClampI32(s_follow_integral_accum, -integral_limit, integral_limit);
    integral_rpm = s_follow_integral_accum / FC_CFG(speed_ki_divisor);
  }
  else
  {
    s_follow_integral_accum = 0;
    integral_rpm = 0;
  }

  max_target_rpm_limit = FC_CFG(max_rpm);
  max_current_limit_ma = FC_CFG(max_current_ma);

  if (abs_error_deg10 <= FC_CFG(settle_zone_deg10))
  {
    max_target_rpm_limit = (abs_error_deg10 * FC_CFG(settle_max_rpm)) /
                           FC_CFG(settle_zone_deg10);
    max_current_limit_ma = (abs_error_deg10 * FC_CFG(settle_max_current_ma)) /
                           FC_CFG(settle_zone_deg10);
  }

  max_target_rpm_limit = FollowControl_ClampI32(max_target_rpm_limit, 0, FC_CFG(max_rpm));
  max_current_limit_ma = FollowControl_ClampI32(max_current_limit_ma, 0, FC_CFG(max_current_ma));

  if ((abs_error_deg10 > FC_CFG(hold_exit_deg10)) &&
      (max_current_limit_ma < FC_CFG(settle_min_current_ma)))
  {
    max_current_limit_ma = FC_CFG(settle_min_current_ma);
  }

  raw_target_rpm = ((int32_t)angle_error_deg10 * FC_CFG(speed_kp_rpm_per_deg)) / 10;
  raw_target_rpm += integral_rpm;
  raw_target_rpm = FollowControl_ClampI32(raw_target_rpm,
                                          -max_target_rpm_limit,
                                          max_target_rpm_limit);

  if ((raw_target_rpm > 0) && (raw_target_rpm < FC_CFG(min_rpm)))
  {
    raw_target_rpm = FC_CFG(min_rpm);
  }
  else if ((raw_target_rpm < 0) && (raw_target_rpm > -FC_CFG(min_rpm)))
  {
    raw_target_rpm = -FC_CFG(min_rpm);
  }

  s_follow_target_rpm_cmd = raw_target_rpm;

  target_rpm = FollowControl_ClampI32(s_follow_target_rpm_cmd,
                                      -MAX_APPLICATION_SPEED_RPM,
                                      MAX_APPLICATION_SPEED_RPM);
  abs_target_rpm = FollowControl_AbsI32(target_rpm);
  motion_alignment = (int32_t)angle_error_deg10 * (int32_t)filtered_speed_rpm;
  urgent_brake = 0U;

  if (abs_error_deg10 >= FC_CFG(hold_exit_deg10))
  {
    if (motion_alignment < 0)
    {
      urgent_brake = 1U;
    }
    else if ((motion_alignment > 0) && (abs_filtered_speed_rpm > abs_target_rpm))
    {
      urgent_brake = 1U;
    }
  }

  position_current_ma = ((int32_t)angle_error_deg10 * FC_CFG(position_current_ma_num)) /
                        FC_CFG(position_current_ma_den);
  position_current_ma = FollowControl_ClampI32(position_current_ma,
                                               -max_current_limit_ma,
                                               max_current_limit_ma);
  abs_position_current_ma = FollowControl_AbsI32(position_current_ma);

  if ((abs_error_deg10 >= FC_CFG(hold_exit_deg10)) &&
      (FollowControl_AbsI32(filtered_speed_rpm) <= FC_CFG(breakaway_speed_window_rpm)) &&
      (abs_position_current_ma < FC_CFG(breakaway_current_ma)))
  {
    breakaway_full_error_deg10 =
        ((FC_CFG(breakaway_current_ma) * FC_CFG(position_current_ma_den)) +
         (FC_CFG(position_current_ma_num) - 1)) /
        FC_CFG(position_current_ma_num);

    if (breakaway_full_error_deg10 <= FC_CFG(hold_exit_deg10))
    {
      breakaway_floor_ma = FC_CFG(breakaway_current_ma);
    }
    else if (abs_error_deg10 >= breakaway_full_error_deg10)
    {
      breakaway_floor_ma = FC_CFG(breakaway_current_ma);
    }
    else
    {
      breakaway_floor_ma =
          ((abs_error_deg10 - FC_CFG(hold_exit_deg10)) * FC_CFG(breakaway_current_ma)) /
          (breakaway_full_error_deg10 - FC_CFG(hold_exit_deg10));
    }

    if (abs_position_current_ma < breakaway_floor_ma)
    {
      position_current_ma = (angle_error_deg10 >= 0) ? breakaway_floor_ma
                                                      : -breakaway_floor_ma;
    }
  }

  damping_current_ma = -filtered_speed_rpm * FC_CFG(damping_current_ma_per_rpm);
  damping_current_ma = FollowControl_ClampI32(damping_current_ma,
                                              -max_current_limit_ma,
                                              max_current_limit_ma);

  target_current_ma = position_current_ma + damping_current_ma;
  target_current_ma = FollowControl_ClampI32(target_current_ma,
                                             -max_current_limit_ma,
                                             max_current_limit_ma);

  if ((abs_error_deg10 <= FC_CFG(hold_exit_deg10)) &&
      (FollowControl_AbsI32(target_current_ma) <= FC_CFG(current_zero_window_ma)))
  {
    target_current_ma = 0;
  }

  if (urgent_brake != 0U)
  {
    s_follow_current_cmd_ma = target_current_ma;
  }
  else
  {
    s_follow_current_cmd_ma = FollowControl_SlewCurrentCommandI32(s_follow_current_cmd_ma,
                                                                  target_current_ma);
  }

  if ((abs_error_deg10 <= FC_CFG(hold_exit_deg10)) &&
      (FollowControl_AbsI32(s_follow_current_cmd_ma) <= FC_CFG(current_zero_window_ma)) &&
      (FollowControl_AbsI32(target_current_ma) <= FC_CFG(current_zero_window_ma)))
  {
    s_follow_current_cmd_ma = 0;
  }

  g_follow_control.target_rpm = (int16_t)target_rpm;
  g_follow_control.filtered_speed_rpm = (int16_t)filtered_speed_rpm;
  g_follow_control.integral_rpm = (int16_t)integral_rpm;
  g_follow_control.active = 1U;
  g_follow_control.hold_active = 0U;
  g_follow_control.reserved0 = (uint8_t)((s_follow_manual_align_hold_active != 0U)
                                             ? FOLLOW_CONTROL_STATUS_MANUAL_ALIGN_HOLD
                                             : ((urgent_brake != 0U)
                                                    ? 2U
                                                    : ((abs_error_deg10 <= FC_CFG(settle_zone_deg10)) ? 1U : 0U)));

  FollowControl_StartOrUpdateCurrentCommand(s_follow_current_cmd_ma);
}

uint16_t FollowControl_GetCommandAngleDeg10(void)
{
  return s_follow_command_angle_deg10;
}

int16_t FollowControl_GetCurrentCommandMilliAmp(void)
{
  return (int16_t)s_follow_current_cmd_ma;
}