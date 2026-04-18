#ifndef USER_SENSORED_H
#define USER_SENSORED_H

#include <stdint.h>

#include "mc_type.h"
#include "speed_pos_fdbk.h"

#ifndef USER_SENSORED_DIRECTION_SIGN
#define USER_SENSORED_DIRECTION_SIGN -1
#endif

#ifndef USER_SENSORED_ALIGN_LOCK_ELEC_DEG10
#define USER_SENSORED_ALIGN_LOCK_ELEC_DEG10 0
#endif

#ifndef USER_SENSORED_ELEC_TRIM_DEG10
#define USER_SENSORED_ELEC_TRIM_DEG10 1800
#endif

#ifndef USER_SENSORED_SPEED_ZERO_WINDOW_RPM
#define USER_SENSORED_SPEED_ZERO_WINDOW_RPM 8
#endif

#ifndef USER_SENSORED_SPEED_LPF_SHIFT
#define USER_SENSORED_SPEED_LPF_SHIFT 1
#endif

#ifndef USER_SENSORED_ALIGN_CURRENT_A
#define USER_SENSORED_ALIGN_CURRENT_A 1
#endif

#ifndef USER_SENSORED_ALIGN_DURATION_MS
#define USER_SENSORED_ALIGN_DURATION_MS 300U
#endif

#ifdef __cplusplus
extern "C" {
#endif

void UserSensored_Init(void);
void UserSensored_ResetRuntime(void);
void UserSensored_InvalidateAlignment(void);
void UserSensored_MediumFrequencyUpdate(void);
void UserSensored_HighFrequencyUpdate(void);
uint8_t UserSensored_CaptureAlignment(void);

SpeednPosFdbk_Handle_t *UserSensored_GetSpeedSensor(void);
uint8_t UserSensored_IsOnline(void);
uint8_t UserSensored_IsAligned(void);
void UserSensored_SetRuntimeDirectionSign(int8_t direction_sign);
void UserSensored_SetRuntimeElectricalTrimDeg10(int16_t trim_deg10);
uint16_t UserSensored_GetRawAngle(void);
uint16_t UserSensored_GetMechanicalAngleDeg10(void);
uint16_t UserSensored_GetElectricalAngleDeg10(void);
int8_t UserSensored_GetRuntimeDirectionSign(void);
uint16_t UserSensored_GetRuntimeElectricalTrimDeg10(void);
uint16_t UserSensored_GetAlignmentMechanicalAngleDeg10(void);
int16_t UserSensored_GetMechanicalSpeedRpm(void);

#ifdef __cplusplus
}
#endif

#endif /* USER_SENSORED_H */