#include <stdint.h>

#include "user_sensored.h"

#include "as5600_follower.h"
#include "follow_tuning_runtime.h"
#include "parameters_conversion.h"
#include "pmsm_motor_parameters.h"

#define US_CFG(field) (FollowTuning_GetProfile()->field)

static int16_t UserSensored_MechanicalSpeedRpmToElSpeedDpp(int16_t mechanical_speed_rpm);
static uint16_t UserSensored_MechanicalToElectricalAngleDeg10(uint16_t mechanical_angle_deg10);

typedef struct
{
  SpeednPosFdbk_Handle_t _Super;
  uint8_t initialized;
  uint8_t online;
  uint8_t aligned;
  uint8_t has_sample;
  uint16_t raw_angle;
  uint16_t mechanical_angle_deg10;
  uint16_t electrical_angle_deg10;
  int8_t runtime_direction_sign;
  uint16_t runtime_electrical_trim_deg10;
  uint16_t alignment_mechanical_angle_deg10;
  int16_t mechanical_speed_rpm;
  int16_t predicted_electrical_angle_s16;
  uint32_t last_sample_tick_ms;
  uint16_t last_mechanical_angle_deg10;
} UserSensored_Handle_t;

static UserSensored_Handle_t s_user_sensored;

static int32_t UserSensored_AbsI32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static uint16_t UserSensored_NormalizeAngleDeg10(int32_t angle_deg10)
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

static int16_t UserSensored_WrapDeltaAngleDeg10(uint16_t current_deg10, uint16_t previous_deg10)
{
  int32_t delta = (int32_t)current_deg10 - (int32_t)previous_deg10;

  while (delta > 1800)
  {
    delta -= 3600;
  }

  while (delta < -1800)
  {
    delta += 3600;
  }

  return (int16_t)delta;
}

static int16_t UserSensored_Deg10ToS16(uint16_t angle_deg10)
{
  const uint32_t normalized = UserSensored_NormalizeAngleDeg10((int32_t)angle_deg10);
  return (int16_t)((normalized * 65536UL) / 3600UL);
}

static void UserSensored_RefreshElectricalState(void)
{
  s_user_sensored._Super.hElSpeedDpp =
      UserSensored_MechanicalSpeedRpmToElSpeedDpp(s_user_sensored.mechanical_speed_rpm);
  s_user_sensored._Super.InstantaneousElSpeedDpp = s_user_sensored._Super.hElSpeedDpp;

  if (s_user_sensored.aligned != 0U)
  {
    s_user_sensored.electrical_angle_deg10 =
        UserSensored_MechanicalToElectricalAngleDeg10(s_user_sensored.mechanical_angle_deg10);
    s_user_sensored.predicted_electrical_angle_s16 =
        UserSensored_Deg10ToS16(s_user_sensored.electrical_angle_deg10);
    s_user_sensored._Super.hElAngle = s_user_sensored.predicted_electrical_angle_s16;
  }
}

static int32_t UserSensored_GetEffectiveDirectionSign(void)
{
  const int32_t runtime_direction_sign = (s_user_sensored.runtime_direction_sign < 0) ? -1 : 1;
  const int32_t configured_direction_sign = (US_CFG(user_sensored_direction_sign) < 0) ? -1 : 1;

  return configured_direction_sign * runtime_direction_sign;
}

static uint16_t UserSensored_RawToMechanicalAngleDeg10(uint16_t raw_angle)
{
  return (uint16_t)((raw_angle * 3600UL) / 4096UL);
}

static int16_t UserSensored_MechanicalSpeedRpmToElSpeedDpp(int16_t mechanical_speed_rpm)
{
  const int32_t electrical_rpm =
      (int32_t)mechanical_speed_rpm * UserSensored_GetEffectiveDirectionSign() * (int32_t)POLE_PAIR_NUM;

  return (int16_t)((electrical_rpm * 65536L) / (60L * (int32_t)TF_REGULATION_RATE));
}

static uint16_t UserSensored_MechanicalToElectricalAngleDeg10(uint16_t mechanical_angle_deg10)
{
  const int32_t mechanical_delta_deg10 =
      (int32_t)UserSensored_NormalizeAngleDeg10((int32_t)mechanical_angle_deg10 -
                                                (int32_t)s_user_sensored.alignment_mechanical_angle_deg10);
  const int32_t electrical_angle_deg10 =
      (mechanical_delta_deg10 * (int32_t)POLE_PAIR_NUM * UserSensored_GetEffectiveDirectionSign()) +
      (int32_t)US_CFG(user_sensored_align_lock_elec_deg10) +
      (int32_t)US_CFG(user_sensored_elec_trim_deg10) +
      (int32_t)s_user_sensored.runtime_electrical_trim_deg10;

  return UserSensored_NormalizeAngleDeg10(electrical_angle_deg10);
}

static void UserSensored_ClearFeedbackState(void)
{
  s_user_sensored._Super.hAvrMecSpeedUnit = 0;
  s_user_sensored._Super.hElSpeedDpp = 0;
  s_user_sensored._Super.InstantaneousElSpeedDpp = 0;
  s_user_sensored.mechanical_speed_rpm = 0;
}

void UserSensored_Init(void)
{
  FollowTuning_Init();

  if (s_user_sensored.initialized != 0U)
  {
    return;
  }

  s_user_sensored._Super.bElToMecRatio = POLE_PAIR_NUM;
  s_user_sensored._Super.SpeedUnit = SPEED_UNIT;
  s_user_sensored._Super.bMaximumSpeedErrorsNumber = M1_SS_MEAS_ERRORS_BEFORE_FAULTS;
  s_user_sensored._Super.hMaxReliableMecSpeedUnit =
      (uint16_t)((115UL * (uint32_t)MAX_APPLICATION_SPEED_UNIT) / 100UL);
  s_user_sensored._Super.hMinReliableMecSpeedUnit = 0U;
  s_user_sensored._Super.hMaxReliableMecAccelUnitP = 65535U;
  s_user_sensored._Super.hMeasurementFrequency = TF_REGULATION_RATE_SCALED;
  s_user_sensored._Super.DPPConvFactor = DPP_CONV_FACTOR;
  s_user_sensored.runtime_direction_sign = 1;
  s_user_sensored.initialized = 1U;
}

void UserSensored_ResetRuntime(void)
{
  UserSensored_Init();
  s_user_sensored.online = 0U;
  s_user_sensored.has_sample = 0U;
  s_user_sensored.last_sample_tick_ms = 0U;
  s_user_sensored.runtime_direction_sign = 1;
  s_user_sensored.runtime_electrical_trim_deg10 = 0U;
  s_user_sensored.predicted_electrical_angle_s16 = UserSensored_Deg10ToS16(s_user_sensored.electrical_angle_deg10);
  s_user_sensored._Super.bSpeedErrorNumber = 0U;
  UserSensored_ClearFeedbackState();
}

void UserSensored_InvalidateAlignment(void)
{
  UserSensored_Init();
  s_user_sensored.aligned = 0U;
  s_user_sensored.alignment_mechanical_angle_deg10 = 0U;
  s_user_sensored.runtime_direction_sign = 1;
  s_user_sensored.runtime_electrical_trim_deg10 = 0U;
  s_user_sensored.electrical_angle_deg10 = 0U;
  s_user_sensored._Super.hElAngle = 0;
  s_user_sensored.predicted_electrical_angle_s16 = 0;
  UserSensored_ClearFeedbackState();
}

void UserSensored_OnTuningProfileChanged(void)
{
  UserSensored_Init();
  UserSensored_RefreshElectricalState();
}

void UserSensored_MediumFrequencyUpdate(void)
{
  uint16_t raw_angle = 0U;
  uint8_t status = 0U;
  uint16_t mechanical_angle_deg10;
  uint32_t now_tick_ms;
  int16_t instant_mechanical_speed_rpm = 0;

  UserSensored_Init();

  if (AS5600_Follower_ReadRawAngleNow(&raw_angle, &status) == 0U)
  {
    s_user_sensored.online = 0U;
    s_user_sensored._Super.bSpeedErrorNumber = s_user_sensored._Super.bMaximumSpeedErrorsNumber;
    g_as5600_follower.online = 0U;
    g_as5600_follower.last_error = 1U;
    g_as5600_follower.error_count++;
    UserSensored_ClearFeedbackState();
    return;
  }

  now_tick_ms = HAL_GetTick();
  mechanical_angle_deg10 = UserSensored_RawToMechanicalAngleDeg10(raw_angle);

  s_user_sensored.online = 1U;
  s_user_sensored.raw_angle = raw_angle;
  s_user_sensored.mechanical_angle_deg10 = mechanical_angle_deg10;
  s_user_sensored._Super.hMecAngle = UserSensored_Deg10ToS16(mechanical_angle_deg10);
  s_user_sensored._Super.wMecAngle = ((int32_t)mechanical_angle_deg10 * 65536L) / 3600L;

  if ((s_user_sensored.has_sample != 0U) && (now_tick_ms != s_user_sensored.last_sample_tick_ms))
  {
    const uint32_t delta_tick_ms = now_tick_ms - s_user_sensored.last_sample_tick_ms;
    const int32_t delta_angle_deg10 =
        (int32_t)UserSensored_WrapDeltaAngleDeg10(mechanical_angle_deg10,
                                                  s_user_sensored.last_mechanical_angle_deg10);

    instant_mechanical_speed_rpm =
        (int16_t)((delta_angle_deg10 * 60000L) / (3600L * (int32_t)delta_tick_ms));
  }

  s_user_sensored.mechanical_speed_rpm +=
      (int16_t)(((int32_t)instant_mechanical_speed_rpm - (int32_t)s_user_sensored.mechanical_speed_rpm) /
                (1 << US_CFG(user_sensored_speed_lpf_shift)));

  if (UserSensored_AbsI32((int32_t)s_user_sensored.mechanical_speed_rpm) <=
      US_CFG(user_sensored_speed_zero_window_rpm))
  {
    s_user_sensored.mechanical_speed_rpm = 0;
  }

  s_user_sensored.last_mechanical_angle_deg10 = mechanical_angle_deg10;
  s_user_sensored.last_sample_tick_ms = now_tick_ms;
  s_user_sensored.has_sample = 1U;

  s_user_sensored._Super.hAvrMecSpeedUnit =
      (int16_t)((s_user_sensored.mechanical_speed_rpm * SPEED_UNIT) / U_RPM);
    UserSensored_RefreshElectricalState();

  s_user_sensored._Super.bSpeedErrorNumber = 0U;

  g_as5600_follower.online = 1U;
  g_as5600_follower.status = status;
  g_as5600_follower.raw_angle = raw_angle;
  g_as5600_follower.angle_deg10 = mechanical_angle_deg10;
  g_as5600_follower.last_update_ms = now_tick_ms;
  g_as5600_follower.last_ok_ms = now_tick_ms;
  g_as5600_follower.sample_count++;
  g_as5600_follower.last_error = 0U;
  g_as5600_follower.magnet_detected = (uint8_t)((status >> 5) & 0x01U);
  g_as5600_follower.magnet_too_weak = (uint8_t)((status >> 4) & 0x01U);
  g_as5600_follower.magnet_too_strong = (uint8_t)((status >> 3) & 0x01U);
}

void UserSensored_HighFrequencyUpdate(void)
{
  UserSensored_Init();

  if ((s_user_sensored.online == 0U) || (s_user_sensored.aligned == 0U))
  {
    return;
  }

  s_user_sensored.predicted_electrical_angle_s16 =
      (int16_t)(s_user_sensored.predicted_electrical_angle_s16 + s_user_sensored._Super.InstantaneousElSpeedDpp);
  s_user_sensored._Super.hElAngle = s_user_sensored.predicted_electrical_angle_s16;
}

uint8_t UserSensored_CaptureAlignment(void)
{
  UserSensored_MediumFrequencyUpdate();

  if (s_user_sensored.online == 0U)
  {
    return 0U;
  }

  s_user_sensored.alignment_mechanical_angle_deg10 = s_user_sensored.mechanical_angle_deg10;
  s_user_sensored.aligned = 1U;
  s_user_sensored.electrical_angle_deg10 =
      UserSensored_MechanicalToElectricalAngleDeg10(s_user_sensored.mechanical_angle_deg10);
  s_user_sensored.predicted_electrical_angle_s16 =
      UserSensored_Deg10ToS16(s_user_sensored.electrical_angle_deg10);
  s_user_sensored._Super.hElAngle = s_user_sensored.predicted_electrical_angle_s16;
  return 1U;
}

SpeednPosFdbk_Handle_t *UserSensored_GetSpeedSensor(void)
{
  UserSensored_Init();
  return &s_user_sensored._Super;
}

uint8_t UserSensored_IsOnline(void)
{
  return s_user_sensored.online;
}

uint8_t UserSensored_IsAligned(void)
{
  return s_user_sensored.aligned;
}

void UserSensored_SetRuntimeDirectionSign(int8_t direction_sign)
{
  UserSensored_Init();
  s_user_sensored.runtime_direction_sign = (direction_sign < 0) ? -1 : 1;
  UserSensored_RefreshElectricalState();
}

void UserSensored_SetRuntimeElectricalTrimDeg10(int16_t trim_deg10)
{
  UserSensored_Init();
  s_user_sensored.runtime_electrical_trim_deg10 = UserSensored_NormalizeAngleDeg10((int32_t)trim_deg10);
  UserSensored_RefreshElectricalState();
}

uint16_t UserSensored_GetRawAngle(void)
{
  return s_user_sensored.raw_angle;
}

uint16_t UserSensored_GetMechanicalAngleDeg10(void)
{
  return s_user_sensored.mechanical_angle_deg10;
}

uint16_t UserSensored_GetElectricalAngleDeg10(void)
{
  return s_user_sensored.electrical_angle_deg10;
}

int8_t UserSensored_GetRuntimeDirectionSign(void)
{
  return (s_user_sensored.runtime_direction_sign < 0) ? -1 : 1;
}

uint16_t UserSensored_GetRuntimeElectricalTrimDeg10(void)
{
  return s_user_sensored.runtime_electrical_trim_deg10;
}

uint16_t UserSensored_GetAlignmentMechanicalAngleDeg10(void)
{
  return s_user_sensored.alignment_mechanical_angle_deg10;
}

int16_t UserSensored_GetMechanicalSpeedRpm(void)
{
  return s_user_sensored.mechanical_speed_rpm;
}