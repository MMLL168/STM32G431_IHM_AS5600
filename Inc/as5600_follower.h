#ifndef AS5600_FOLLOWER_H
#define AS5600_FOLLOWER_H

#include "as5600_knob.h"

extern volatile As5600KnobSnapshot_t g_as5600_follower;

void AS5600_Follower_Init(void);
void AS5600_Follower_Task(void);
uint8_t AS5600_Follower_ReadRawAngleNow(uint16_t *raw_angle, uint8_t *status);

#endif /* AS5600_FOLLOWER_H */