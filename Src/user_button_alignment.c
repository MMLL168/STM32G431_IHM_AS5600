#include "user_button_alignment.h"

#include "follow_control.h"
#include "main.h"

#define USER_BUTTON_ALIGN_RELEASE_DEBOUNCE_MS 30U

typedef struct
{
  uint8_t state;
  uint8_t release_candidate;
  uint32_t release_candidate_tick_ms;
} UserButtonAlignmentContext_t;

static UserButtonAlignmentContext_t s_user_button_alignment = {0};

static uint8_t UserButtonAlignment_IsButtonPressed(void)
{
  return (HAL_GPIO_ReadPin(Start_Stop_GPIO_Port, Start_Stop_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

void UserButtonAlignment_Init(void)
{
  s_user_button_alignment.state = USER_BUTTON_ALIGNMENT_STATE_IDLE;
  s_user_button_alignment.release_candidate = 0U;
  s_user_button_alignment.release_candidate_tick_ms = 0U;
}

void UserButtonAlignment_ArmStartOnRelease(void)
{
  if (s_user_button_alignment.state == USER_BUTTON_ALIGNMENT_STATE_WAIT_RELEASE)
  {
    return;
  }

  s_user_button_alignment.state = USER_BUTTON_ALIGNMENT_STATE_WAIT_RELEASE;
  s_user_button_alignment.release_candidate = 0U;
  s_user_button_alignment.release_candidate_tick_ms = 0U;
}

void UserButtonAlignment_Cancel(void)
{
  s_user_button_alignment.state = USER_BUTTON_ALIGNMENT_STATE_IDLE;
  s_user_button_alignment.release_candidate = 0U;
  s_user_button_alignment.release_candidate_tick_ms = 0U;
}

void UserButtonAlignment_Task(void)
{
  const uint32_t now_ms = HAL_GetTick();

  if (s_user_button_alignment.state != USER_BUTTON_ALIGNMENT_STATE_WAIT_RELEASE)
  {
    return;
  }

  if (UserButtonAlignment_IsButtonPressed() != 0U)
  {
    s_user_button_alignment.release_candidate = 0U;
    s_user_button_alignment.release_candidate_tick_ms = 0U;
    return;
  }

  if (s_user_button_alignment.release_candidate == 0U)
  {
    s_user_button_alignment.release_candidate = 1U;
    s_user_button_alignment.release_candidate_tick_ms = now_ms;
    return;
  }

  if ((uint32_t)(now_ms - s_user_button_alignment.release_candidate_tick_ms) <
      USER_BUTTON_ALIGN_RELEASE_DEBOUNCE_MS)
  {
    return;
  }

  FollowControl_EndManualAlignHold();
  UserButtonAlignment_Cancel();
}

uint8_t UserButtonAlignment_IsWaitReleaseActive(void)
{
  return (s_user_button_alignment.state == USER_BUTTON_ALIGNMENT_STATE_WAIT_RELEASE) ? 1U : 0U;
}

uint8_t UserButtonAlignment_GetState(void)
{
  return s_user_button_alignment.state;
}