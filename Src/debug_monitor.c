#include "debug_monitor.h"

#if defined(DEBUG)

#include "as5600_follower.h"
#include "as5600_knob.h"
#include "follow_control.h"
#include "follow_tuning_runtime.h"
#include "main.h"
#include "motorcontrol.h"
#include "user_sensored.h"

#define DEBUG_MONITOR_MAGIC 0x4442474Du
#define DEBUG_MONITOR_PERIOD_MS 50u

volatile DebugMonitorSnapshot_t g_debug_monitor = {0};

void DebugMonitor_Init(void)
{
  g_debug_monitor.magic = DEBUG_MONITOR_MAGIC;
  g_debug_monitor.heartbeat = 0u;
  g_debug_monitor.uptime_ms = 0u;
  g_debug_monitor.last_sample_ms = 0u;
  g_debug_monitor.state = 0u;
  g_debug_monitor.command_state = 0u;
  g_debug_monitor.current_faults = 0u;
  g_debug_monitor.occurred_faults = 0u;
  g_debug_monitor.knob_last_update_ms = 0u;
  g_debug_monitor.knob_last_ok_ms = 0u;
  g_debug_monitor.knob_sample_count = 0u;
  g_debug_monitor.knob_error_count = 0u;
  g_debug_monitor.knob_raw = 0u;
  g_debug_monitor.knob_angle_deg10 = 0u;
  g_debug_monitor.follower_last_update_ms = 0u;
  g_debug_monitor.follower_last_ok_ms = 0u;
  g_debug_monitor.follower_sample_count = 0u;
  g_debug_monitor.follower_error_count = 0u;
  g_debug_monitor.follower_raw = 0u;
  g_debug_monitor.follower_angle_deg10 = 0u;
  g_debug_monitor.knob_ready = 0u;
  g_debug_monitor.knob_online = 0u;
  g_debug_monitor.knob_status = 0u;
  g_debug_monitor.knob_magnet_detected = 0u;
  g_debug_monitor.knob_magnet_too_weak = 0u;
  g_debug_monitor.knob_magnet_too_strong = 0u;
  g_debug_monitor.knob_last_error = 0u;
  g_debug_monitor.knob_reserved = 0u;
  g_debug_monitor.follower_ready = 0u;
  g_debug_monitor.follower_online = 0u;
  g_debug_monitor.follower_status = 0u;
  g_debug_monitor.follower_magnet_detected = 0u;
  g_debug_monitor.follower_magnet_too_weak = 0u;
  g_debug_monitor.follower_magnet_too_strong = 0u;
  g_debug_monitor.follower_last_error = 0u;
  g_debug_monitor.follower_reserved = 0u;
  g_debug_monitor.follow_target_angle_deg10 = 0u;
  g_debug_monitor.follow_actual_angle_deg10 = 0u;
  g_debug_monitor.follow_angle_error_deg10 = 0;
  g_debug_monitor.follow_target_rpm = 0;
  g_debug_monitor.follow_filtered_speed_rpm = 0;
  g_debug_monitor.follow_integral_rpm = 0;
  g_debug_monitor.follow_current_command_ma = 0;
  g_debug_monitor.follow_test_mode = 0;
  g_debug_monitor.follow_probe_phase_index = 0;
  g_debug_monitor.follow_active = 0u;
  g_debug_monitor.follow_hold_active = 1u;
  g_debug_monitor.follow_source_online = 0u;
  g_debug_monitor.follow_follower_online = 0u;
  g_debug_monitor.follow_reserved = 0u;
  g_debug_monitor.sensored_online = 0u;
  g_debug_monitor.sensored_aligned = 0u;
  g_debug_monitor.sensored_raw = 0u;
  g_debug_monitor.sensored_mech_angle_deg10 = 0u;
  g_debug_monitor.sensored_elec_angle_deg10 = 0u;
  g_debug_monitor.sensored_align_angle_deg10 = 0u;
  g_debug_monitor.sensored_speed_rpm = 0;
  g_debug_monitor.sensored_runtime_direction_sign = 0;
  g_debug_monitor.sensored_runtime_electrical_trim_deg10 = 0;
  g_debug_monitor.sensored_reserved = 0u;
  g_debug_monitor.speed_ref_rpm = 0.0f;
  g_debug_monitor.speed_avg_rpm = 0.0f;
  g_debug_monitor.power_w = 0.0f;
  g_debug_monitor.iq_a = 0.0f;
  g_debug_monitor.id_a = 0.0f;
  g_debug_monitor.iq_ref_a = 0.0f;
  g_debug_monitor.id_ref_a = 0.0f;
}

void DebugMonitor_Task(void)
{
  static uint32_t next_sample_ms = 0u;
  const uint32_t now = HAL_GetTick();

  g_debug_monitor.heartbeat++;
  g_debug_monitor.uptime_ms = now;

  if (now < next_sample_ms)
  {
    return;
  }

  next_sample_ms = now + DEBUG_MONITOR_PERIOD_MS;

  g_debug_monitor.last_sample_ms = now;
  g_debug_monitor.state = (uint16_t)MC_GetSTMStateMotor1();
  g_debug_monitor.command_state = (uint16_t)MC_GetCommandStateMotor1();
  g_debug_monitor.current_faults = MC_GetCurrentFaultsMotor1();
  g_debug_monitor.occurred_faults = MC_GetOccurredFaultsMotor1();
  g_debug_monitor.knob_last_update_ms = g_as5600_knob.last_update_ms;
  g_debug_monitor.knob_last_ok_ms = g_as5600_knob.last_ok_ms;
  g_debug_monitor.knob_sample_count = g_as5600_knob.sample_count;
  g_debug_monitor.knob_error_count = g_as5600_knob.error_count;
  g_debug_monitor.knob_raw = g_as5600_knob.raw_angle;
  g_debug_monitor.knob_angle_deg10 = g_as5600_knob.angle_deg10;
  g_debug_monitor.follower_last_update_ms = g_as5600_follower.last_update_ms;
  g_debug_monitor.follower_last_ok_ms = g_as5600_follower.last_ok_ms;
  g_debug_monitor.follower_sample_count = g_as5600_follower.sample_count;
  g_debug_monitor.follower_error_count = g_as5600_follower.error_count;
  g_debug_monitor.follower_raw = g_as5600_follower.raw_angle;
  g_debug_monitor.follower_angle_deg10 = g_as5600_follower.angle_deg10;
  g_debug_monitor.knob_ready = g_as5600_knob.ready;
  g_debug_monitor.knob_online = g_as5600_knob.online;
  g_debug_monitor.knob_status = g_as5600_knob.status;
  g_debug_monitor.knob_magnet_detected = g_as5600_knob.magnet_detected;
  g_debug_monitor.knob_magnet_too_weak = g_as5600_knob.magnet_too_weak;
  g_debug_monitor.knob_magnet_too_strong = g_as5600_knob.magnet_too_strong;
  g_debug_monitor.knob_last_error = g_as5600_knob.last_error;
  g_debug_monitor.follower_ready = g_as5600_follower.ready;
  g_debug_monitor.follower_online = g_as5600_follower.online;
  g_debug_monitor.follower_status = g_as5600_follower.status;
  g_debug_monitor.follower_magnet_detected = g_as5600_follower.magnet_detected;
  g_debug_monitor.follower_magnet_too_weak = g_as5600_follower.magnet_too_weak;
  g_debug_monitor.follower_magnet_too_strong = g_as5600_follower.magnet_too_strong;
  g_debug_monitor.follower_last_error = g_as5600_follower.last_error;
  g_debug_monitor.follower_reserved = (UserSensored_GetRuntimeDirectionSign() > 0) ? 1U : 0U;
  g_debug_monitor.follow_target_angle_deg10 = g_follow_control.target_angle_deg10;
  g_debug_monitor.follow_actual_angle_deg10 = g_follow_control.actual_angle_deg10;
  g_debug_monitor.follow_angle_error_deg10 = g_follow_control.angle_error_deg10;
  g_debug_monitor.follow_target_rpm = g_follow_control.target_rpm;
  g_debug_monitor.follow_filtered_speed_rpm = g_follow_control.filtered_speed_rpm;
  g_debug_monitor.follow_integral_rpm = g_follow_control.integral_rpm;
  g_debug_monitor.follow_current_command_ma = FollowControl_GetCurrentCommandMilliAmp();
  g_debug_monitor.follow_test_mode = (int16_t)FollowTuning_GetProfile()->test_mode;
  g_debug_monitor.follow_probe_phase_index =
      (g_debug_monitor.follow_test_mode == FOLLOW_CONTROL_TEST_MODE_DIRECTION_PROBE)
          ? (int16_t)g_follow_control.reserved0
          : -1;
  g_debug_monitor.follow_active = g_follow_control.active;
  g_debug_monitor.follow_hold_active = g_follow_control.hold_active;
  g_debug_monitor.follow_source_online = g_follow_control.source_online;
  g_debug_monitor.follow_follower_online = g_follow_control.follower_online;
  g_debug_monitor.follow_reserved = g_follow_control.reserved0;
  g_debug_monitor.sensored_online = UserSensored_IsOnline();
  g_debug_monitor.sensored_aligned = UserSensored_IsAligned();
  g_debug_monitor.sensored_raw = UserSensored_GetRawAngle();
  g_debug_monitor.sensored_mech_angle_deg10 = UserSensored_GetMechanicalAngleDeg10();
  g_debug_monitor.sensored_elec_angle_deg10 = UserSensored_GetElectricalAngleDeg10();
  g_debug_monitor.sensored_align_angle_deg10 = UserSensored_GetAlignmentMechanicalAngleDeg10();
  g_debug_monitor.sensored_speed_rpm = UserSensored_GetMechanicalSpeedRpm();
  g_debug_monitor.sensored_runtime_direction_sign = UserSensored_GetRuntimeDirectionSign();
  g_debug_monitor.sensored_runtime_electrical_trim_deg10 = UserSensored_GetRuntimeElectricalTrimDeg10();
  g_debug_monitor.sensored_reserved = UserSensored_GetRuntimeElectricalTrimDeg10();
  g_debug_monitor.speed_ref_rpm = MC_GetMecSpeedReferenceMotor1_F();
  g_debug_monitor.speed_avg_rpm = MC_GetAverageMecSpeedMotor1_F();
  g_debug_monitor.power_w = MC_GetAveragePowerMotor1_F();

  {
    const qd_f_t iqd = MC_GetIqdMotor1_F();
    const qd_f_t iqd_ref = MC_GetIqdrefMotor1_F();

    g_debug_monitor.iq_a = iqd.q;
    g_debug_monitor.id_a = iqd.d;
    g_debug_monitor.iq_ref_a = iqd_ref.q;
    g_debug_monitor.id_ref_a = iqd_ref.d;
  }
}

#endif /* DEBUG */
