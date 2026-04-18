#include "follow_tuning_runtime.h"

#include <string.h>

#include "as5600_follower.h"
#include "as5600_knob.h"
#include "follow_control.h"
#include "tuning_config.h"
#include "user_sensored.h"

#define FOLLOW_TUNING_TEST_MODE_COUNT 4
#define FOLLOW_TUNING_PROBE_PHASE_COUNT 8

static FollowTuningProfile_t s_profiles[2];
static volatile uint8_t s_active_profile_index = 0U;
static uint8_t s_initialized = 0U;

static int32_t FollowTuning_ClampI32(int32_t value, int32_t min_value, int32_t max_value)
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

static int32_t FollowTuning_NormalizeSign(int32_t value)
{
  return (value < 0) ? -1 : 1;
}

static void FollowTuning_LoadDefaultProfile(FollowTuningProfile_t *profile)
{
  *profile = (FollowTuningProfile_t){
      .test_mode = FOLLOW_CONTROL_TEST_MODE,
      .fixed_target_offset_deg10 = FOLLOW_CONTROL_FIXED_TARGET_OFFSET_DEG10,
      .step_amplitude_deg10 = FOLLOW_CONTROL_STEP_AMPLITUDE_DEG10,
      .step_dwell_ms = FOLLOW_CONTROL_STEP_DWELL_MS,
      .step_start_positive = FOLLOW_CONTROL_STEP_START_POSITIVE,
      .hold_enter_deg10 = FOLLOW_CONTROL_HOLD_ENTER_DEG10,
      .hold_exit_deg10 = FOLLOW_CONTROL_HOLD_EXIT_DEG10,
      .speed_zero_window_rpm = FOLLOW_CONTROL_SPEED_ZERO_WINDOW_RPM,
      .speed_kp_rpm_per_deg = FOLLOW_CONTROL_SPEED_KP_RPM_PER_DEG,
      .position_current_ma_num = FOLLOW_CONTROL_POSITION_CURRENT_MA_NUM,
      .position_current_ma_den = FOLLOW_CONTROL_POSITION_CURRENT_MA_DEN,
      .damping_current_ma_per_rpm = FOLLOW_CONTROL_DAMPING_CURRENT_MA_PER_RPM,
      .breakaway_current_ma = FOLLOW_CONTROL_BREAKAWAY_CURRENT_MA,
      .breakaway_speed_window_rpm = FOLLOW_CONTROL_BREAKAWAY_SPEED_WINDOW_RPM,
      .speed_ki_divisor = FOLLOW_CONTROL_SPEED_KI_DIVISOR,
      .speed_ki_enable = FOLLOW_CONTROL_SPEED_KI_ENABLE,
      .speed_kd_rpm_per_rpm = FOLLOW_CONTROL_SPEED_KD_RPM_PER_RPM,
      .speed_to_current_kp_ma_per_rpm = FOLLOW_CONTROL_SPEED_TO_CURRENT_KP_MA_PER_RPM,
      .outer_angle_filter_shift = FOLLOW_CONTROL_OUTER_ANGLE_FILTER_SHIFT,
      .target_angle_slew_min_deg10_per_tick = FOLLOW_CONTROL_TARGET_ANGLE_SLEW_MIN_DEG10_PER_TICK,
      .target_angle_slew_max_deg10_per_tick = FOLLOW_CONTROL_TARGET_ANGLE_SLEW_MAX_DEG10_PER_TICK,
      .target_angle_slew_divisor = FOLLOW_CONTROL_TARGET_ANGLE_SLEW_DIVISOR,
      .target_angle_slew_accel_deg10_per_tick = FOLLOW_CONTROL_TARGET_ANGLE_SLEW_ACCEL_DEG10_PER_TICK,
      .target_angle_slew_decel_deg10_per_tick = FOLLOW_CONTROL_TARGET_ANGLE_SLEW_DECEL_DEG10_PER_TICK,
      .settle_zone_deg10 = FOLLOW_CONTROL_SETTLE_ZONE_DEG10,
      .settle_max_rpm = FOLLOW_CONTROL_SETTLE_MAX_RPM,
      .settle_max_current_ma = FOLLOW_CONTROL_SETTLE_MAX_CURRENT_MA,
      .settle_min_current_ma = FOLLOW_CONTROL_SETTLE_MIN_CURRENT_MA,
      .max_rpm = FOLLOW_CONTROL_MAX_RPM,
      .min_rpm = FOLLOW_CONTROL_MIN_RPM,
      .max_current_ma = FOLLOW_CONTROL_MAX_CURRENT_MA,
      .current_zero_window_ma = FOLLOW_CONTROL_CURRENT_ZERO_WINDOW_MA,
      .source_direction_sign = FOLLOW_CONTROL_SOURCE_DIRECTION_SIGN,
      .follower_direction_sign = FOLLOW_CONTROL_FOLLOWER_DIRECTION_SIGN,
      .direction_probe_target_abs_rpm = FOLLOW_CONTROL_DIRECTION_PROBE_TARGET_ABS_RPM,
      .direction_probe_max_current_ma = FOLLOW_CONTROL_DIRECTION_PROBE_MAX_CURRENT_MA,
      .direction_probe_speed_to_current_kp_ma_per_rpm = FOLLOW_CONTROL_DIRECTION_PROBE_SPEED_TO_CURRENT_KP_MA_PER_RPM,
      .direction_probe_output_sign = FOLLOW_CONTROL_DIRECTION_PROBE_OUTPUT_SIGN,
      .direction_probe_phase_duration_ms = FOLLOW_CONTROL_DIRECTION_PROBE_PHASE_DURATION_MS,
      .direction_probe_sweep_enable = FOLLOW_CONTROL_DIRECTION_PROBE_SWEEP_ENABLE,
      .direction_probe_fixed_phase_index = FOLLOW_CONTROL_DIRECTION_PROBE_FIXED_PHASE_INDEX,
      .user_sensored_direction_sign = USER_SENSORED_DIRECTION_SIGN,
      .user_sensored_align_lock_elec_deg10 = USER_SENSORED_ALIGN_LOCK_ELEC_DEG10,
      .user_sensored_elec_trim_deg10 = USER_SENSORED_ELEC_TRIM_DEG10,
      .user_sensored_speed_zero_window_rpm = USER_SENSORED_SPEED_ZERO_WINDOW_RPM,
      .user_sensored_speed_lpf_shift = USER_SENSORED_SPEED_LPF_SHIFT,
      .user_sensored_align_current_a = USER_SENSORED_ALIGN_CURRENT_A,
      .user_sensored_align_duration_ms = USER_SENSORED_ALIGN_DURATION_MS,
  };
}

static void FollowTuning_SanitizeProfile(FollowTuningProfile_t *profile)
{
  if ((profile->test_mode < 0) || (profile->test_mode >= FOLLOW_TUNING_TEST_MODE_COUNT))
  {
    profile->test_mode = FOLLOW_CONTROL_TEST_MODE_NORMAL_FOLLOW;
  }

  profile->step_start_positive = (profile->step_start_positive != 0) ? 1 : 0;
  profile->speed_ki_enable = (profile->speed_ki_enable != 0) ? 1 : 0;
  profile->direction_probe_sweep_enable = (profile->direction_probe_sweep_enable != 0) ? 1 : 0;

  profile->hold_enter_deg10 = FollowTuning_ClampI32(profile->hold_enter_deg10, 0, 3600);
  profile->hold_exit_deg10 = FollowTuning_ClampI32(profile->hold_exit_deg10, profile->hold_enter_deg10, 3600);
  profile->speed_zero_window_rpm = FollowTuning_ClampI32(profile->speed_zero_window_rpm, 0, 100000);
  profile->breakaway_current_ma = FollowTuning_ClampI32(profile->breakaway_current_ma, 0, 100000);
  profile->breakaway_speed_window_rpm = FollowTuning_ClampI32(profile->breakaway_speed_window_rpm, 0, 100000);
  profile->position_current_ma_den = FollowTuning_ClampI32(profile->position_current_ma_den, 1, 1000000);
  profile->speed_ki_divisor = FollowTuning_ClampI32(profile->speed_ki_divisor, 1, 1000000000);
  profile->outer_angle_filter_shift = FollowTuning_ClampI32(profile->outer_angle_filter_shift, 0, 7);
  profile->target_angle_slew_min_deg10_per_tick =
      FollowTuning_ClampI32(profile->target_angle_slew_min_deg10_per_tick, 0, 3600);
  profile->target_angle_slew_max_deg10_per_tick =
      FollowTuning_ClampI32(profile->target_angle_slew_max_deg10_per_tick,
                            profile->target_angle_slew_min_deg10_per_tick,
                            3600);
  profile->target_angle_slew_divisor = FollowTuning_ClampI32(profile->target_angle_slew_divisor, 1, 1000000);
  profile->target_angle_slew_accel_deg10_per_tick =
      FollowTuning_ClampI32(profile->target_angle_slew_accel_deg10_per_tick, 1, 3600);
  profile->target_angle_slew_decel_deg10_per_tick =
      FollowTuning_ClampI32(profile->target_angle_slew_decel_deg10_per_tick, 1, 3600);
  profile->settle_zone_deg10 = FollowTuning_ClampI32(profile->settle_zone_deg10, 1, 3600);
  profile->max_rpm = FollowTuning_ClampI32(profile->max_rpm, 0, 100000);
  profile->min_rpm = FollowTuning_ClampI32(profile->min_rpm, 0, profile->max_rpm);
  profile->settle_max_rpm = FollowTuning_ClampI32(profile->settle_max_rpm, 0, profile->max_rpm);
  profile->max_current_ma = FollowTuning_ClampI32(profile->max_current_ma, 0, 100000);
  profile->settle_max_current_ma = FollowTuning_ClampI32(profile->settle_max_current_ma, 0, profile->max_current_ma);
  profile->settle_min_current_ma = FollowTuning_ClampI32(profile->settle_min_current_ma, 0, profile->max_current_ma);
  profile->current_zero_window_ma = FollowTuning_ClampI32(profile->current_zero_window_ma, 0, profile->max_current_ma);
  profile->step_dwell_ms = FollowTuning_ClampI32(profile->step_dwell_ms, 1, 600000);
  profile->source_direction_sign = FollowTuning_NormalizeSign(profile->source_direction_sign);
  profile->follower_direction_sign = FollowTuning_NormalizeSign(profile->follower_direction_sign);
  profile->direction_probe_target_abs_rpm =
      FollowTuning_ClampI32(profile->direction_probe_target_abs_rpm, 0, 100000);
  profile->direction_probe_max_current_ma =
      FollowTuning_ClampI32(profile->direction_probe_max_current_ma, 0, 100000);
  profile->direction_probe_speed_to_current_kp_ma_per_rpm =
      FollowTuning_ClampI32(profile->direction_probe_speed_to_current_kp_ma_per_rpm, 0, 100000);
  profile->direction_probe_output_sign = FollowTuning_NormalizeSign(profile->direction_probe_output_sign);
  profile->direction_probe_phase_duration_ms =
      FollowTuning_ClampI32(profile->direction_probe_phase_duration_ms, 1, 600000);
  profile->direction_probe_fixed_phase_index =
      FollowTuning_ClampI32(profile->direction_probe_fixed_phase_index, 0, FOLLOW_TUNING_PROBE_PHASE_COUNT - 1);
  profile->user_sensored_direction_sign = FollowTuning_NormalizeSign(profile->user_sensored_direction_sign);
  profile->user_sensored_speed_zero_window_rpm =
      FollowTuning_ClampI32(profile->user_sensored_speed_zero_window_rpm, 0, 100000);
  profile->user_sensored_speed_lpf_shift =
      FollowTuning_ClampI32(profile->user_sensored_speed_lpf_shift, 0, 7);
  profile->user_sensored_align_current_a = FollowTuning_ClampI32(profile->user_sensored_align_current_a, 0, 1000);
  profile->user_sensored_align_duration_ms =
      FollowTuning_ClampI32(profile->user_sensored_align_duration_ms, 1, 600000);
}

void FollowTuning_Init(void)
{
  FollowTuningProfile_t default_profile;

  if (s_initialized != 0U)
  {
    return;
  }

  FollowTuning_LoadDefaultProfile(&default_profile);
  FollowTuning_SanitizeProfile(&default_profile);
  s_profiles[0] = default_profile;
  s_profiles[1] = default_profile;
  s_active_profile_index = 0U;
  s_initialized = 1U;
}

const FollowTuningProfile_t *FollowTuning_GetProfile(void)
{
  FollowTuning_Init();
  return &s_profiles[s_active_profile_index & 0x1U];
}

void FollowTuning_GetProfileCopy(FollowTuningProfile_t *profile)
{
  if (profile == (FollowTuningProfile_t *)0)
  {
    return;
  }

  *profile = *FollowTuning_GetProfile();
}

void FollowTuning_SetProfile(const FollowTuningProfile_t *profile)
{
  uint8_t inactive_index;

  if (profile == (const FollowTuningProfile_t *)0)
  {
    return;
  }

  FollowTuning_Init();
  inactive_index = (uint8_t)((s_active_profile_index + 1U) & 0x1U);
  s_profiles[inactive_index] = *profile;
  FollowTuning_SanitizeProfile(&s_profiles[inactive_index]);
  s_active_profile_index = inactive_index;
}

void FollowTuning_ResetProfile(void)
{
  FollowTuningProfile_t default_profile;

  FollowTuning_LoadDefaultProfile(&default_profile);
  FollowTuning_SanitizeProfile(&default_profile);
  FollowTuning_SetProfile(&default_profile);
}

void FollowTuning_GetHello(FollowTuningHello_t *hello)
{
  if (hello == (FollowTuningHello_t *)0)
  {
    return;
  }

  hello->protocol_version = FOLLOW_TUNING_PROTOCOL_VERSION;
  hello->profile_size = (int32_t)sizeof(FollowTuningProfile_t);
  hello->snapshot_size = (int32_t)sizeof(FollowTuningSnapshot_t);
  hello->active_test_mode = FollowTuning_GetProfile()->test_mode;
}

void FollowTuning_GetSnapshot(FollowTuningSnapshot_t *snapshot)
{
  if (snapshot == (FollowTuningSnapshot_t *)0)
  {
    return;
  }

  snapshot->protocol_version = FOLLOW_TUNING_PROTOCOL_VERSION;
  snapshot->test_mode = FollowTuning_GetProfile()->test_mode;
  snapshot->command_angle_deg10 = (int32_t)FollowControl_GetCommandAngleDeg10();
  snapshot->target_angle_deg10 = (int32_t)g_follow_control.target_angle_deg10;
  snapshot->actual_angle_deg10 = (int32_t)g_follow_control.actual_angle_deg10;
  snapshot->angle_error_deg10 = (int32_t)g_follow_control.angle_error_deg10;
  snapshot->target_rpm = (int32_t)g_follow_control.target_rpm;
  snapshot->filtered_speed_rpm = (int32_t)g_follow_control.filtered_speed_rpm;
  snapshot->integral_rpm = (int32_t)g_follow_control.integral_rpm;
  snapshot->current_command_ma = (int32_t)FollowControl_GetCurrentCommandMilliAmp();
  snapshot->active = (int32_t)g_follow_control.active;
  snapshot->hold_active = (int32_t)g_follow_control.hold_active;
  snapshot->source_online = (int32_t)g_follow_control.source_online;
  snapshot->follower_online = (int32_t)g_follow_control.follower_online;
  snapshot->settle_active = (int32_t)g_follow_control.reserved0;
  snapshot->knob_online = (int32_t)g_as5600_knob.online;
  snapshot->knob_angle_deg10 = (int32_t)g_as5600_knob.angle_deg10;
  snapshot->knob_raw_angle = (int32_t)g_as5600_knob.raw_angle;
  snapshot->follower_raw_angle = (int32_t)g_as5600_follower.raw_angle;
  snapshot->follower_angle_deg10 = (int32_t)g_as5600_follower.angle_deg10;
  snapshot->follower_status = (int32_t)g_as5600_follower.status;
  snapshot->follower_magnet_detected = (int32_t)g_as5600_follower.magnet_detected;
  snapshot->follower_magnet_too_weak = (int32_t)g_as5600_follower.magnet_too_weak;
  snapshot->follower_magnet_too_strong = (int32_t)g_as5600_follower.magnet_too_strong;
  snapshot->sensored_online = (int32_t)UserSensored_IsOnline();
  snapshot->sensored_aligned = (int32_t)UserSensored_IsAligned();
  snapshot->sensored_mechanical_angle_deg10 = (int32_t)UserSensored_GetMechanicalAngleDeg10();
  snapshot->sensored_electrical_angle_deg10 = (int32_t)UserSensored_GetElectricalAngleDeg10();
  snapshot->sensored_mechanical_speed_rpm = (int32_t)UserSensored_GetMechanicalSpeedRpm();
  snapshot->sensored_runtime_direction_sign = (int32_t)UserSensored_GetRuntimeDirectionSign();
  snapshot->sensored_runtime_electrical_trim_deg10 = (int32_t)UserSensored_GetRuntimeElectricalTrimDeg10();
}