#include "main.h"
#include "mc_config.h"
#include "mc_config_common.h"
#include "mc_interface.h"
#include "mc_math.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "pwm_common.h"
#include "regular_conversion_manager.h"
#include "user_sensored.h"

#define M1_CHARGE_BOOT_CAP_TICKS (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES (uint32_t)(0.000 * ((uint32_t)PWM_PERIOD_CYCLES / 2U))

static uint32_t s_alignment_start_tick_ms = 0U;

void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);

static int16_t UserSensored_GetLockElectricalAngleS16(void)
{
  return (int16_t)((USER_SENSORED_ALIGN_LOCK_ELEC_DEG10 * 65536UL) / 3600UL);
}

static qd_t UserSensored_GetAlignmentCurrentReference(void)
{
  qd_t iqd_ref = {0};

  iqd_ref.q = 0;
  iqd_ref.d = (int16_t)(USER_SENSORED_ALIGN_CURRENT_A * CURRENT_CONV_FACTOR);
  return iqd_ref;
}

static void UserSensored_StopProcessing(uint8_t motor)
{
  R3_2_SwitchOffPWM(pwmcHandle[motor]);
  FOC_Clear(motor);
  STC_Clear(pSTC[motor]);
  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;
}

static void UserSensored_EnterRunState(void)
{
  FOCVars[M1].Iqdref = STC_GetDefaultIqdref(pSTC[M1]);
  FOCVars[M1].UserIdref = FOCVars[M1].Iqdref.d;
  STC_SetSpeedSensor(pSTC[M1], UserSensored_GetSpeedSensor());
  FOC_InitAdditionalMethods(M1);
  FOC_CalcCurrRef(M1);
  STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]);
  MCI_ExecBufferedCommands(&Mci[M1]);
  Mci[M1].State = RUN;
}

static void UserSensored_BeginAlignment(void)
{
  const qd_t iqd_ref = UserSensored_GetAlignmentCurrentReference();

  VSS_Clear(&VirtualSpeedSensorM1);
  VSS_SetElAngle(&VirtualSpeedSensorM1, UserSensored_GetLockElectricalAngleS16());
  STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);
  FOCVars[M1].bDriveInput = EXTERNAL;
  FOCVars[M1].Iqdref = iqd_ref;
  FOCVars[M1].UserIdref = iqd_ref.d;
  s_alignment_start_tick_ms = HAL_GetTick();
  PWMC_SwitchOnPWM(pwmcHandle[M1]);
  Mci[M1].State = START;
}

static uint16_t UserSensored_FOC_CurrControllerM1(void)
{
  qd_t iqd, vqd;
  ab_t iab;
  alphabeta_t ialphabeta, valphabeta;
  int16_t h_el_angle;
  uint16_t h_code_error = MC_NO_FAULTS;
  SpeednPosFdbk_Handle_t *speed_handle;

  speed_handle = STC_GetSpeedSensor(pSTC[M1]);
  h_el_angle = SPD_GetElAngle(speed_handle);
  h_el_angle += SPD_GetInstElSpeedDpp(speed_handle) * PARK_ANGLE_COMPENSATION_FACTOR;
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &iab);
  ialphabeta = MCM_Clarke(iab);
  iqd = MCM_Park(ialphabeta, h_el_angle);

  if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
  {
    vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - iqd.q);
    vqd.d = PI_Controller(pPIDId[M1], (int32_t)(FOCVars[M1].Iqdref.d) - iqd.d);
  }
  else
  {
    vqd.q = 0;
    vqd.d = 0;
  }

  vqd = Circle_Limitation(&CircleLimitationM1, vqd);
  h_el_angle += SPD_GetInstElSpeedDpp(speed_handle) * REV_PARK_ANGLE_COMPENSATION_FACTOR;
  valphabeta = MCM_Rev_Park(vqd, h_el_angle);

  if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
  {
    h_code_error = PWMC_SetPhaseVoltage(pwmcHandle[M1], valphabeta);
  }

  FOCVars[M1].Vqd = vqd;
  FOCVars[M1].Iab = iab;
  FOCVars[M1].Ialphabeta = ialphabeta;
  FOCVars[M1].Iqd = iqd;
  FOCVars[M1].Valphabeta = valphabeta;
  FOCVars[M1].hElAngle = h_el_angle;

  return h_code_error;
}

void TSK_MediumFrequencyTaskM1(void)
{
  PQD_CalcElMotorPower(pMPM[M1]);
  UserSensored_MediumFrequencyUpdate();

  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {
        case IDLE:
          if ((MCI_START == Mci[M1].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand))
          {
            if (pwmcHandle[M1]->offsetCalibStatus == false)
            {
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_START);
              Mci[M1].State = OFFSET_CALIB;
            }
            else
            {
              pwmcHandle[M1]->OffCalibrWaitTimeCounter = 1u;
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC);
              R3_2_TurnOnLowSides(pwmcHandle[M1], M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
              Mci[M1].State = CHARGE_BOOT_CAP;
            }
          }
          break;

        case OFFSET_CALIB:
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            UserSensored_StopProcessing(M1);
          }
          else if (PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC))
          {
            if (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand)
            {
              FOC_Clear(M1);
              STC_Clear(pSTC[M1]);
              Mci[M1].DirectCommand = MCI_NO_COMMAND;
              Mci[M1].State = IDLE;
            }
            else
            {
              R3_2_TurnOnLowSides(pwmcHandle[M1], M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
              Mci[M1].State = CHARGE_BOOT_CAP;
            }
          }
          break;

        case CHARGE_BOOT_CAP:
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            UserSensored_StopProcessing(M1);
          }
          else if (TSK_ChargeBootCapDelayHasElapsedM1())
          {
            R3_2_SwitchOffPWM(pwmcHandle[M1]);
            FOCVars[M1].bDriveInput = EXTERNAL;
            FOC_Clear(M1);
            UserSensored_ResetRuntime();
            UserSensored_MediumFrequencyUpdate();

            if ((UserSensored_IsAligned() != 0U) && (UserSensored_IsOnline() != 0U))
            {
              STC_SetSpeedSensor(pSTC[M1], UserSensored_GetSpeedSensor());
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
              UserSensored_EnterRunState();
            }
            else
            {
              UserSensored_BeginAlignment();
            }
          }
          break;

        case START:
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            UserSensored_StopProcessing(M1);
          }
          else
          {
            const qd_t iqd_ref = UserSensored_GetAlignmentCurrentReference();

            FOCVars[M1].bDriveInput = EXTERNAL;
            FOCVars[M1].Iqdref = iqd_ref;
            FOCVars[M1].UserIdref = iqd_ref.d;

            if ((HAL_GetTick() - s_alignment_start_tick_ms) >= USER_SENSORED_ALIGN_DURATION_MS)
            {
              if (UserSensored_CaptureAlignment() == 0U)
              {
                MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
              }
              else
              {
                UserSensored_EnterRunState();
              }
            }
          }
          break;

        case RUN:
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            UserSensored_StopProcessing(M1);
          }
          else
          {
            MCI_ExecBufferedCommands(&Mci[M1]);
            FOC_CalcCurrRef(M1);
            if (!SPD_Check(UserSensored_GetSpeedSensor()))
            {
              MCI_FaultProcessing(&Mci[M1], MC_SPEED_FDBK, 0);
            }
          }
          break;

        case STOP:
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {
            STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);
            VSS_Clear(&VirtualSpeedSensorM1);
            UserSensored_ResetRuntime();
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          break;

        case FAULT_OVER:
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          break;

        case FAULT_NOW:
          Mci[M1].State = FAULT_OVER;
          break;

        default:
          break;
      }
    }
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }
}

uint8_t FOC_HighFrequencyTask(uint8_t bMotorNbr)
{
  uint16_t h_foc_return;

  RCM_ReadOngoingConv();
  RCM_ExecNextConv();

  if ((Mci[M1].State != IDLE) && (Mci[M1].State != FAULT_NOW) && (Mci[M1].State != FAULT_OVER))
  {
    UserSensored_HighFrequencyUpdate();
  }

  h_foc_return = UserSensored_FOC_CurrControllerM1();
  if (h_foc_return == MC_DURATION)
  {
    MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
  }

  return bMotorNbr;
}