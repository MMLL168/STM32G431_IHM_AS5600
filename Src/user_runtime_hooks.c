#include "user_runtime_hooks.h"

#include "as5600_follower.h"
#include "as5600_knob.h"
#include "debug_monitor.h"
#include "follow_control.h"
#include "follow_tuning_mcp.h"
#include "follow_tuning_runtime.h"
#include "user_sensored.h"

void UserRuntime_BootHook(void)
{
  FollowTuning_Init();
  AS5600_Knob_Init();
  AS5600_Follower_Init();
  UserSensored_Init();
  FollowControl_Init();
  FollowTuningMcp_Init();
  DebugMonitor_Init();
}

void UserRuntime_MainLoopHook(void)
{
  AS5600_Knob_Task();
  FollowControl_Task();
  DebugMonitor_Task();
}
