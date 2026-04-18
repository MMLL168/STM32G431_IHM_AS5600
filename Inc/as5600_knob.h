#ifndef AS5600_KNOB_H
#define AS5600_KNOB_H

#include <stdint.h>

typedef struct
{
  uint32_t last_update_ms;
  uint32_t last_ok_ms;
  uint32_t sample_count;
  uint32_t error_count;
  uint16_t raw_angle;
  uint16_t angle_deg10;
  uint8_t ready;
  uint8_t online;
  uint8_t status;
  uint8_t magnet_detected;
  uint8_t magnet_too_weak;
  uint8_t magnet_too_strong;
  uint8_t last_error;
  uint8_t reserved;
} As5600KnobSnapshot_t;

extern volatile As5600KnobSnapshot_t g_as5600_knob;

void AS5600_Knob_Init(void);
void AS5600_Knob_Task(void);

#endif /* AS5600_KNOB_H */
