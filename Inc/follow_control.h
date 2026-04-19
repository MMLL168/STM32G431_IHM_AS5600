#ifndef FOLLOW_CONTROL_H
#define FOLLOW_CONTROL_H

#include <stdint.h>

typedef struct
{
  uint16_t target_angle_deg10;
  uint16_t actual_angle_deg10;
  int16_t angle_error_deg10;
  int16_t target_rpm;
  int16_t filtered_speed_rpm;
  int16_t integral_rpm;
  uint8_t active;
  uint8_t hold_active;
  uint8_t source_online;
  uint8_t follower_online;
  uint8_t reserved0;
} FollowControlSnapshot_t;

extern volatile FollowControlSnapshot_t g_follow_control;

void FollowControl_Init(void);
void FollowControl_Task(void);
void FollowControl_OnTuningProfileChanged(void);
uint8_t FollowControl_BeginManualAlignHold(void);
void FollowControl_EndManualAlignHold(void);
uint16_t FollowControl_GetCommandAngleDeg10(void);
int16_t FollowControl_GetCurrentCommandMilliAmp(void);

#endif /* FOLLOW_CONTROL_H */