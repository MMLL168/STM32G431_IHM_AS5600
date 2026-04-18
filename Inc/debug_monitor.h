#ifndef DEBUG_MONITOR_H
#define DEBUG_MONITOR_H

#include <stdint.h>

#if defined(DEBUG)

typedef struct
{
  uint32_t magic;
  uint32_t heartbeat;
  uint32_t uptime_ms;
  uint32_t last_sample_ms;
  uint16_t state;
  uint16_t command_state;
  uint16_t current_faults;
  uint16_t occurred_faults;
  uint32_t knob_last_update_ms;
  uint32_t knob_last_ok_ms;
  uint32_t knob_sample_count;
  uint32_t knob_error_count;
  uint16_t knob_raw;
  uint16_t knob_angle_deg10;
  uint32_t follower_last_update_ms;
  uint32_t follower_last_ok_ms;
  uint32_t follower_sample_count;
  uint32_t follower_error_count;
  uint16_t follower_raw;
  uint16_t follower_angle_deg10;
  uint8_t knob_ready;
  uint8_t knob_online;
  uint8_t knob_status;
  uint8_t knob_magnet_detected;
  uint8_t knob_magnet_too_weak;
  uint8_t knob_magnet_too_strong;
  uint8_t knob_last_error;
  uint8_t knob_reserved;
  uint8_t follower_ready;
  uint8_t follower_online;
  uint8_t follower_status;
  uint8_t follower_magnet_detected;
  uint8_t follower_magnet_too_weak;
  uint8_t follower_magnet_too_strong;
  uint8_t follower_last_error;
  uint8_t follower_reserved;
  uint16_t follow_target_angle_deg10;
  uint16_t follow_actual_angle_deg10;
  int16_t follow_angle_error_deg10;
  int16_t follow_target_rpm;
  int16_t follow_filtered_speed_rpm;
  int16_t follow_integral_rpm;
  int16_t follow_current_command_ma;
  int16_t follow_test_mode;
  int16_t follow_probe_phase_index;
  uint8_t follow_active;
  uint8_t follow_hold_active;
  uint8_t follow_source_online;
  uint8_t follow_follower_online;
  uint8_t follow_reserved;
  uint8_t sensored_online;
  uint8_t sensored_aligned;
  uint16_t sensored_raw;
  uint16_t sensored_mech_angle_deg10;
  uint16_t sensored_elec_angle_deg10;
  uint16_t sensored_align_angle_deg10;
  int16_t sensored_speed_rpm;
  int16_t sensored_runtime_direction_sign;
  int16_t sensored_runtime_electrical_trim_deg10;
  uint16_t sensored_reserved;
  float speed_ref_rpm;
  float speed_avg_rpm;
  float power_w;
  float iq_a;
  float id_a;
  float iq_ref_a;
  float id_ref_a;
} DebugMonitorSnapshot_t;

extern volatile DebugMonitorSnapshot_t g_debug_monitor;

void DebugMonitor_Init(void);
void DebugMonitor_Task(void);

#else

static inline void DebugMonitor_Init(void)
{
}

static inline void DebugMonitor_Task(void)
{
}

#endif /* DEBUG */

#endif /* DEBUG_MONITOR_H */
