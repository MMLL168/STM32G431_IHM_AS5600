#ifndef USER_BUTTON_ALIGNMENT_H
#define USER_BUTTON_ALIGNMENT_H

#include <stdint.h>

#define USER_BUTTON_ALIGNMENT_STATE_IDLE 0U
#define USER_BUTTON_ALIGNMENT_STATE_WAIT_RELEASE 1U

void UserButtonAlignment_Init(void);
void UserButtonAlignment_Task(void);
void UserButtonAlignment_ArmStartOnRelease(void);
void UserButtonAlignment_Cancel(void);
uint8_t UserButtonAlignment_IsWaitReleaseActive(void);
uint8_t UserButtonAlignment_GetState(void);

#endif /* USER_BUTTON_ALIGNMENT_H */