#include "as5600_follower.h"

#include "main.h"

#define AS5600_FOLLOWER_I2C_ADDR (0x36U << 1)
#define AS5600_FOLLOWER_REG_STATUS 0x0BU
#define AS5600_FOLLOWER_REG_ANGLE 0x0EU
#define AS5600_FOLLOWER_SDA_PIN GPIO_PIN_8
#define AS5600_FOLLOWER_SCL_PIN GPIO_PIN_9
#define AS5600_FOLLOWER_GPIO_PORT GPIOC
#define AS5600_FOLLOWER_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define AS5600_FOLLOWER_POLL_PERIOD_MS 10U
#define AS5600_FOLLOWER_RETRY_PERIOD_MS 500U
#define AS5600_FOLLOWER_HALF_CYCLE_US 5U
#define AS5600_FOLLOWER_TIMEOUT_US 100U

enum
{
  AS5600_FOLLOWER_ERROR_NONE = 0U,
  AS5600_FOLLOWER_ERROR_NOT_READY = 1U,
  AS5600_FOLLOWER_ERROR_STATUS_READ = 2U,
  AS5600_FOLLOWER_ERROR_ANGLE_READ = 3U
};

volatile As5600KnobSnapshot_t g_as5600_follower = {0};

static void AS5600_Follower_DelayUs(uint32_t delay_us)
{
  static uint8_t dwt_ready = 0U;

  if (delay_us == 0U)
  {
    return;
  }

  if (dwt_ready == 0U)
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_ready = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? 1U : 0U;
  }

  if (dwt_ready != 0U)
  {
    const uint32_t start = DWT->CYCCNT;
    const uint32_t cycles = (SystemCoreClock / 1000000U) * delay_us;

    while ((DWT->CYCCNT - start) < cycles)
    {
    }

    return;
  }

  for (volatile uint32_t i = 0U; i < (delay_us * 16U); ++i)
  {
    __NOP();
  }
}

static void AS5600_Follower_SdaHigh(void)
{
  HAL_GPIO_WritePin(AS5600_FOLLOWER_GPIO_PORT, AS5600_FOLLOWER_SDA_PIN, GPIO_PIN_SET);
}

static void AS5600_Follower_SdaLow(void)
{
  HAL_GPIO_WritePin(AS5600_FOLLOWER_GPIO_PORT, AS5600_FOLLOWER_SDA_PIN, GPIO_PIN_RESET);
}

static void AS5600_Follower_SclHigh(void)
{
  HAL_GPIO_WritePin(AS5600_FOLLOWER_GPIO_PORT, AS5600_FOLLOWER_SCL_PIN, GPIO_PIN_SET);
}

static void AS5600_Follower_SclLow(void)
{
  HAL_GPIO_WritePin(AS5600_FOLLOWER_GPIO_PORT, AS5600_FOLLOWER_SCL_PIN, GPIO_PIN_RESET);
}

static uint8_t AS5600_Follower_ReadSda(void)
{
  return (HAL_GPIO_ReadPin(AS5600_FOLLOWER_GPIO_PORT, AS5600_FOLLOWER_SDA_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

static uint8_t AS5600_Follower_ReadScl(void)
{
  return (HAL_GPIO_ReadPin(AS5600_FOLLOWER_GPIO_PORT, AS5600_FOLLOWER_SCL_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

static uint8_t AS5600_Follower_WaitForSclHigh(void)
{
  uint32_t elapsed_us = 0U;

  while ((AS5600_Follower_ReadScl() == 0U) && (elapsed_us < AS5600_FOLLOWER_TIMEOUT_US))
  {
    AS5600_Follower_DelayUs(1U);
    elapsed_us++;
  }

  return (AS5600_Follower_ReadScl() != 0U) ? 1U : 0U;
}

static void AS5600_Follower_Start(void)
{
  AS5600_Follower_SdaHigh();
  AS5600_Follower_SclHigh();
  AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
  AS5600_Follower_SdaLow();
  AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
  AS5600_Follower_SclLow();
}

static void AS5600_Follower_Stop(void)
{
  AS5600_Follower_SdaLow();
  AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
  AS5600_Follower_SclHigh();
  (void)AS5600_Follower_WaitForSclHigh();
  AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
  AS5600_Follower_SdaHigh();
  AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
}

static uint8_t AS5600_Follower_WriteByte(uint8_t value)
{
  for (uint32_t bit = 0U; bit < 8U; ++bit)
  {
    if ((value & 0x80U) != 0U)
    {
      AS5600_Follower_SdaHigh();
    }
    else
    {
      AS5600_Follower_SdaLow();
    }

    AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
    AS5600_Follower_SclHigh();
    if (AS5600_Follower_WaitForSclHigh() == 0U)
    {
      AS5600_Follower_SclLow();
      return 0U;
    }
    AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
    AS5600_Follower_SclLow();
    value <<= 1;
  }

  AS5600_Follower_SdaHigh();
  AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
  AS5600_Follower_SclHigh();
  if (AS5600_Follower_WaitForSclHigh() == 0U)
  {
    AS5600_Follower_SclLow();
    return 0U;
  }

  {
    const uint8_t ack = (AS5600_Follower_ReadSda() == 0U) ? 1U : 0U;

    AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
    AS5600_Follower_SclLow();
    return ack;
  }
}

static uint8_t AS5600_Follower_ReadByte(uint8_t send_ack)
{
  uint8_t value = 0U;

  AS5600_Follower_SdaHigh();

  for (uint32_t bit = 0U; bit < 8U; ++bit)
  {
    value <<= 1;
    AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
    AS5600_Follower_SclHigh();
    if (AS5600_Follower_WaitForSclHigh() != 0U)
    {
      value |= AS5600_Follower_ReadSda();
    }
    AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
    AS5600_Follower_SclLow();
  }

  if (send_ack != 0U)
  {
    AS5600_Follower_SdaLow();
  }
  else
  {
    AS5600_Follower_SdaHigh();
  }

  AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
  AS5600_Follower_SclHigh();
  (void)AS5600_Follower_WaitForSclHigh();
  AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
  AS5600_Follower_SclLow();
  AS5600_Follower_SdaHigh();

  return value;
}

static void AS5600_Follower_BusRecovery(void)
{
  AS5600_Follower_SdaHigh();

  for (uint32_t i = 0U; i < 9U; ++i)
  {
    AS5600_Follower_SclHigh();
    AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
    AS5600_Follower_SclLow();
    AS5600_Follower_DelayUs(AS5600_FOLLOWER_HALF_CYCLE_US);
  }

  AS5600_Follower_Stop();
}

static uint8_t AS5600_Follower_ReadRegister(uint8_t reg, uint8_t *value)
{
  if (value == NULL)
  {
    return 0U;
  }

  AS5600_Follower_Start();
  if (AS5600_Follower_WriteByte((uint8_t)(AS5600_FOLLOWER_I2C_ADDR | 0U)) == 0U)
  {
    AS5600_Follower_Stop();
    return 0U;
  }

  if (AS5600_Follower_WriteByte(reg) == 0U)
  {
    AS5600_Follower_Stop();
    return 0U;
  }

  AS5600_Follower_Start();
  if (AS5600_Follower_WriteByte((uint8_t)(AS5600_FOLLOWER_I2C_ADDR | 1U)) == 0U)
  {
    AS5600_Follower_Stop();
    return 0U;
  }

  *value = AS5600_Follower_ReadByte(0U);
  AS5600_Follower_Stop();
  return 1U;
}

static uint8_t AS5600_Follower_ReadRawAngle(uint16_t *raw_angle)
{
  uint8_t msb = 0U;
  uint8_t lsb = 0U;

  if (raw_angle == NULL)
  {
    return 0U;
  }

  AS5600_Follower_Start();
  if (AS5600_Follower_WriteByte((uint8_t)(AS5600_FOLLOWER_I2C_ADDR | 0U)) == 0U)
  {
    AS5600_Follower_Stop();
    return 0U;
  }

  if (AS5600_Follower_WriteByte(AS5600_FOLLOWER_REG_ANGLE) == 0U)
  {
    AS5600_Follower_Stop();
    return 0U;
  }

  AS5600_Follower_Start();
  if (AS5600_Follower_WriteByte((uint8_t)(AS5600_FOLLOWER_I2C_ADDR | 1U)) == 0U)
  {
    AS5600_Follower_Stop();
    return 0U;
  }

  msb = AS5600_Follower_ReadByte(1U);
  lsb = AS5600_Follower_ReadByte(0U);
  AS5600_Follower_Stop();

  *raw_angle = (uint16_t)((((uint16_t)msb) << 8) | lsb) & 0x0FFFU;
  return 1U;
}

static uint16_t AS5600_Follower_RawToAngleDeg10(uint16_t raw_angle)
{
  return (uint16_t)((((uint32_t)raw_angle) * 3600U) / 4096U);
}

static uint8_t AS5600_Follower_Probe(void)
{
  uint8_t ack = 0U;

  AS5600_Follower_Start();
  ack = AS5600_Follower_WriteByte((uint8_t)(AS5600_FOLLOWER_I2C_ADDR | 0U));
  AS5600_Follower_Stop();
  return ack;
}

uint8_t AS5600_Follower_ReadRawAngleNow(uint16_t *raw_angle, uint8_t *status)
{
  uint8_t local_status = 0U;
  uint16_t local_raw_angle = 0U;

  if ((raw_angle == NULL) || (g_as5600_follower.ready == 0U))
  {
    return 0U;
  }

  if (AS5600_Follower_Probe() == 0U)
  {
    return 0U;
  }

  if (AS5600_Follower_ReadRegister(AS5600_FOLLOWER_REG_STATUS, &local_status) == 0U)
  {
    return 0U;
  }

  if (AS5600_Follower_ReadRawAngle(&local_raw_angle) == 0U)
  {
    return 0U;
  }

  *raw_angle = local_raw_angle;
  if (status != NULL)
  {
    *status = local_status;
  }

  return 1U;
}

void AS5600_Follower_Init(void)
{
  GPIO_InitTypeDef gpio_init_struct = {0};

  if (g_as5600_follower.ready != 0U)
  {
    return;
  }

  AS5600_FOLLOWER_GPIO_CLK_ENABLE();

  gpio_init_struct.Pin = AS5600_FOLLOWER_SDA_PIN | AS5600_FOLLOWER_SCL_PIN;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;
  gpio_init_struct.Pull = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(AS5600_FOLLOWER_GPIO_PORT, &gpio_init_struct);

  AS5600_Follower_SdaHigh();
  AS5600_Follower_SclHigh();
  AS5600_Follower_BusRecovery();

  g_as5600_follower.ready = 1U;
  g_as5600_follower.online = 0U;
  g_as5600_follower.status = 0U;
  g_as5600_follower.magnet_detected = 0U;
  g_as5600_follower.magnet_too_weak = 0U;
  g_as5600_follower.magnet_too_strong = 0U;
  g_as5600_follower.last_error = AS5600_FOLLOWER_ERROR_NONE;
}

void AS5600_Follower_Task(void)
{
  static uint32_t next_poll_ms = 0U;
  const uint32_t now = HAL_GetTick();
  uint8_t status = 0U;
  uint16_t raw_angle = 0U;

  if (g_as5600_follower.ready == 0U)
  {
    return;
  }

  if (now < next_poll_ms)
  {
    return;
  }

  next_poll_ms = now + ((g_as5600_follower.online != 0U) ? AS5600_FOLLOWER_POLL_PERIOD_MS
                                                         : AS5600_FOLLOWER_RETRY_PERIOD_MS);

  g_as5600_follower.sample_count++;
  g_as5600_follower.last_update_ms = now;

  if (AS5600_Follower_Probe() == 0U)
  {
    g_as5600_follower.online = 0U;
    g_as5600_follower.last_error = AS5600_FOLLOWER_ERROR_NOT_READY;
    g_as5600_follower.error_count++;
    return;
  }

  if (AS5600_Follower_ReadRegister(AS5600_FOLLOWER_REG_STATUS, &status) == 0U)
  {
    g_as5600_follower.online = 0U;
    g_as5600_follower.last_error = AS5600_FOLLOWER_ERROR_STATUS_READ;
    g_as5600_follower.error_count++;
    return;
  }

  if (AS5600_Follower_ReadRawAngle(&raw_angle) == 0U)
  {
    g_as5600_follower.online = 0U;
    g_as5600_follower.last_error = AS5600_FOLLOWER_ERROR_ANGLE_READ;
    g_as5600_follower.error_count++;
    return;
  }

  g_as5600_follower.online = 1U;
  g_as5600_follower.status = status;
  g_as5600_follower.raw_angle = raw_angle;
  g_as5600_follower.angle_deg10 = AS5600_Follower_RawToAngleDeg10(raw_angle);
  g_as5600_follower.magnet_detected = (uint8_t)((status >> 5) & 0x01U);
  g_as5600_follower.magnet_too_weak = (uint8_t)((status >> 4) & 0x01U);
  g_as5600_follower.magnet_too_strong = (uint8_t)((status >> 3) & 0x01U);
  g_as5600_follower.last_ok_ms = now;
  g_as5600_follower.last_error = AS5600_FOLLOWER_ERROR_NONE;
}