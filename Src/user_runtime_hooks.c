#include "user_runtime_hooks.h"

#include "as5600_follower.h"
#include "as5600_knob.h"
#include "debug_monitor.h"
#include "follow_control.h"
#include "follow_tuning_mcp.h"
#include "follow_tuning_runtime.h"
#include "mc_api.h"
#include "user_button_alignment.h"
#include "user_sensored.h"

void UserRuntime_BootHook(void)
{
  FollowTuning_Init();
  AS5600_Knob_Init();
  AS5600_Follower_Init();
  UserButtonAlignment_Init();
  UserSensored_Init();
  FollowControl_Init();
  FollowTuningMcp_Init();
  DebugMonitor_Init();
}

void UserRuntime_MainLoopHook(void)
{
  AS5600_Knob_Task();
  UserButtonAlignment_Task();
  FollowControl_Task();
  DebugMonitor_Task();
}

void UI_HandleStartStopButton_cb(void)
{
  const MCI_State_t state = MC_GetSTMStateMotor1();

  if ((state == RUN) &&
      (FollowTuning_GetProfile()->test_mode == FOLLOW_CONTROL_TEST_MODE_NORMAL_FOLLOW))
  {
    if (FollowControl_BeginManualAlignHold() != 0U)
    {
      UserButtonAlignment_ArmStartOnRelease();
    }
  }
  else if (state == IDLE)
  {
    UserButtonAlignment_Cancel();
    (void)MC_StartMotor1();
  }
  else
  {
    UserButtonAlignment_Cancel();
    (void)MC_StopMotor1();
  }
}
