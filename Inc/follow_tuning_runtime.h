#ifndef FOLLOW_TUNING_RUNTIME_H
#define FOLLOW_TUNING_RUNTIME_H

#include <stdint.h>

#define FOLLOW_TUNING_PROTOCOL_VERSION 1
#define FOLLOW_TUNING_MCP_CALLBACK_ID 0U

#define FOLLOW_TUNING_MCP_OPCODE_PING 0U
#define FOLLOW_TUNING_MCP_OPCODE_GET_PROFILE 1U
#define FOLLOW_TUNING_MCP_OPCODE_SET_PROFILE 2U
#define FOLLOW_TUNING_MCP_OPCODE_GET_SNAPSHOT 3U
#define FOLLOW_TUNING_MCP_OPCODE_RESET_PROFILE 4U

typedef struct
{
  int32_t test_mode;
  int32_t fixed_target_offset_deg10;
  int32_t step_amplitude_deg10;
  int32_t step_dwell_ms;
  int32_t step_start_positive;
  int32_t hold_enter_deg10;
  int32_t hold_exit_deg10;
  int32_t speed_zero_window_rpm;
  int32_t speed_kp_rpm_per_deg;
  int32_t position_current_ma_num;
  int32_t position_current_ma_den;
  int32_t damping_current_ma_per_rpm;
  int32_t breakaway_current_ma;
  int32_t breakaway_speed_window_rpm;
  int32_t speed_ki_divisor;
  int32_t speed_ki_enable;
  int32_t speed_kd_rpm_per_rpm;
  int32_t speed_to_current_kp_ma_per_rpm;
  int32_t outer_angle_filter_shift;
  int32_t target_angle_slew_min_deg10_per_tick;
  int32_t target_angle_slew_max_deg10_per_tick;
  int32_t target_angle_slew_divisor;
  int32_t target_angle_slew_accel_deg10_per_tick;
  int32_t target_angle_slew_decel_deg10_per_tick;
  int32_t settle_zone_deg10;
  int32_t settle_max_rpm;
  int32_t settle_max_current_ma;
  int32_t settle_min_current_ma;
  int32_t max_rpm;
  int32_t min_rpm;
  int32_t max_current_ma;
  int32_t current_zero_window_ma;
  int32_t source_direction_sign;
  int32_t follower_direction_sign;
  int32_t direction_probe_target_abs_rpm;
  int32_t direction_probe_max_current_ma;
  int32_t direction_probe_speed_to_current_kp_ma_per_rpm;
  int32_t direction_probe_output_sign;
  int32_t direction_probe_phase_duration_ms;
  int32_t direction_probe_sweep_enable;
  int32_t direction_probe_fixed_phase_index;
  int32_t user_sensored_direction_sign;
  int32_t user_sensored_align_lock_elec_deg10;
  int32_t user_sensored_elec_trim_deg10;
  int32_t user_sensored_speed_zero_window_rpm;
  int32_t user_sensored_speed_lpf_shift;
  int32_t user_sensored_align_current_a;
  int32_t user_sensored_align_duration_ms;
} FollowTuningProfile_t;

typedef struct
{
  int32_t protocol_version;
  int32_t profile_size;
  int32_t snapshot_size;
  int32_t active_test_mode;
} FollowTuningHello_t;

typedef struct
{
  int32_t protocol_version;
  int32_t test_mode;
  int32_t command_angle_deg10;
  int32_t target_angle_deg10;
  int32_t actual_angle_deg10;
  int32_t angle_error_deg10;
  int32_t target_rpm;
  int32_t filtered_speed_rpm;
  int32_t integral_rpm;
  int32_t current_command_ma;
  int32_t active;
  int32_t hold_active;
  int32_t source_online;
  int32_t follower_online;
  int32_t settle_active;
  int32_t knob_online;
  int32_t knob_angle_deg10;
  int32_t knob_raw_angle;
  int32_t follower_raw_angle;
  int32_t follower_angle_deg10;
  int32_t follower_status;
  int32_t follower_magnet_detected;
  int32_t follower_magnet_too_weak;
  int32_t follower_magnet_too_strong;
  int32_t sensored_online;
  int32_t sensored_aligned;
  int32_t sensored_mechanical_angle_deg10;
  int32_t sensored_electrical_angle_deg10;
  int32_t sensored_mechanical_speed_rpm;
  int32_t sensored_runtime_direction_sign;
  int32_t sensored_runtime_electrical_trim_deg10;
} FollowTuningSnapshot_t;

void FollowTuning_Init(void);
const FollowTuningProfile_t *FollowTuning_GetProfile(void);
void FollowTuning_GetProfileCopy(FollowTuningProfile_t *profile);
void FollowTuning_SetProfile(const FollowTuningProfile_t *profile);
void FollowTuning_ResetProfile(void);
void FollowTuning_GetHello(FollowTuningHello_t *hello);
void FollowTuning_GetSnapshot(FollowTuningSnapshot_t *snapshot);

#endif /* FOLLOW_TUNING_RUNTIME_H */