#include "as5600_knob.h"

#include "main.h"
#include "stm32g4xx_hal_i2c.h"

#define AS5600_KNOB_I2C_INSTANCE I2C1
#define AS5600_KNOB_I2C_TIMING 0x10C0ECFFU
#define AS5600_KNOB_I2C_ADDR (0x36U << 1)
#define AS5600_KNOB_REG_STATUS 0x0BU
#define AS5600_KNOB_REG_ANGLE 0x0EU
#define AS5600_KNOB_SDA_PIN GPIO_PIN_7
#define AS5600_KNOB_SCL_PIN GPIO_PIN_8
#define AS5600_KNOB_GPIO_PORT GPIOB
#define AS5600_KNOB_POLL_PERIOD_MS 10U
#define AS5600_KNOB_RETRY_PERIOD_MS 500U
#define AS5600_KNOB_I2C_TIMEOUT_MS 2U

enum
{
  AS5600_KNOB_ERROR_NONE = 0U,
  AS5600_KNOB_ERROR_NOT_READY = 1U,
  AS5600_KNOB_ERROR_STATUS_READ = 2U,
  AS5600_KNOB_ERROR_ANGLE_READ = 3U
};

static I2C_HandleTypeDef s_hi2c1;

volatile As5600KnobSnapshot_t g_as5600_knob = {0};

static uint8_t AS5600_Knob_ReadByte(uint8_t reg, uint8_t *value)
{
  if ((value == NULL) || (g_as5600_knob.ready == 0U))
  {
    return 0U;
  }

  return (HAL_I2C_Mem_Read(&s_hi2c1,
                           AS5600_KNOB_I2C_ADDR,
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           value,
                           1U,
                           AS5600_KNOB_I2C_TIMEOUT_MS) == HAL_OK)
             ? 1U
             : 0U;
}

static uint8_t AS5600_Knob_ReadRawAngle(uint16_t *raw_angle)
{
  uint8_t rx_data[2] = {0};

  if ((raw_angle == NULL) || (g_as5600_knob.ready == 0U))
  {
    return 0U;
  }

  if (HAL_I2C_Mem_Read(&s_hi2c1,
                       AS5600_KNOB_I2C_ADDR,
                       AS5600_KNOB_REG_ANGLE,
                       I2C_MEMADD_SIZE_8BIT,
                       rx_data,
                       2U,
                       AS5600_KNOB_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return 0U;
  }

  *raw_angle = (uint16_t)((((uint16_t)rx_data[0]) << 8) | rx_data[1]) & 0x0FFFU;
  return 1U;
}

static uint16_t AS5600_Knob_RawToAngleDeg10(uint16_t raw_angle)
{
  return (uint16_t)((((uint32_t)raw_angle) * 3600U) / 4096U);
}

void AS5600_Knob_Init(void)
{
  if (g_as5600_knob.ready != 0U)
  {
    return;
  }

  s_hi2c1.Instance = AS5600_KNOB_I2C_INSTANCE;
  s_hi2c1.Init.Timing = AS5600_KNOB_I2C_TIMING;
  s_hi2c1.Init.OwnAddress1 = 0U;
  s_hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  s_hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  s_hi2c1.Init.OwnAddress2 = 0U;
  s_hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  s_hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  s_hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&s_hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&s_hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&s_hi2c1, 0U) != HAL_OK)
  {
    Error_Handler();
  }

  g_as5600_knob.ready = 1U;
  g_as5600_knob.online = 0U;
  g_as5600_knob.status = 0U;
  g_as5600_knob.magnet_detected = 0U;
  g_as5600_knob.magnet_too_weak = 0U;
  g_as5600_knob.magnet_too_strong = 0U;
  g_as5600_knob.last_error = AS5600_KNOB_ERROR_NONE;
}

void AS5600_Knob_Task(void)
{
  static uint32_t next_poll_ms = 0U;
  const uint32_t now = HAL_GetTick();
  uint8_t status = 0U;
  uint16_t raw_angle = 0U;

  if (g_as5600_knob.ready == 0U)
  {
    return;
  }

  if (now < next_poll_ms)
  {
    return;
  }

  next_poll_ms = now + ((g_as5600_knob.online != 0U) ? AS5600_KNOB_POLL_PERIOD_MS
                                                     : AS5600_KNOB_RETRY_PERIOD_MS);

  g_as5600_knob.sample_count++;
  g_as5600_knob.last_update_ms = now;

  if (HAL_I2C_IsDeviceReady(&s_hi2c1,
                            AS5600_KNOB_I2C_ADDR,
                            1U,
                            AS5600_KNOB_I2C_TIMEOUT_MS) != HAL_OK)
  {
    g_as5600_knob.online = 0U;
    g_as5600_knob.last_error = AS5600_KNOB_ERROR_NOT_READY;
    g_as5600_knob.error_count++;
    return;
  }

  if (AS5600_Knob_ReadByte(AS5600_KNOB_REG_STATUS, &status) == 0U)
  {
    g_as5600_knob.online = 0U;
    g_as5600_knob.last_error = AS5600_KNOB_ERROR_STATUS_READ;
    g_as5600_knob.error_count++;
    return;
  }

  if (AS5600_Knob_ReadRawAngle(&raw_angle) == 0U)
  {
    g_as5600_knob.online = 0U;
    g_as5600_knob.last_error = AS5600_KNOB_ERROR_ANGLE_READ;
    g_as5600_knob.error_count++;
    return;
  }

  g_as5600_knob.online = 1U;
  g_as5600_knob.status = status;
  g_as5600_knob.raw_angle = raw_angle;
  g_as5600_knob.angle_deg10 = AS5600_Knob_RawToAngleDeg10(raw_angle);
  g_as5600_knob.magnet_detected = (uint8_t)((status >> 5) & 0x01U);
  g_as5600_knob.magnet_too_weak = (uint8_t)((status >> 4) & 0x01U);
  g_as5600_knob.magnet_too_strong = (uint8_t)((status >> 3) & 0x01U);
  g_as5600_knob.last_ok_ms = now;
  g_as5600_knob.last_error = AS5600_KNOB_ERROR_NONE;
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef gpio_init_struct = {0};
  RCC_PeriphCLKInitTypeDef periph_clk_init = {0};

  if (hi2c->Instance != AS5600_KNOB_I2C_INSTANCE)
  {
    return;
  }

  periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  periph_clk_init.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio_init_struct.Pin = AS5600_KNOB_SDA_PIN | AS5600_KNOB_SCL_PIN;
  gpio_init_struct.Mode = GPIO_MODE_AF_OD;
  gpio_init_struct.Pull = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init_struct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(AS5600_KNOB_GPIO_PORT, &gpio_init_struct);

  __HAL_RCC_I2C1_CLK_ENABLE();
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != AS5600_KNOB_I2C_INSTANCE)
  {
    return;
  }

  __HAL_RCC_I2C1_CLK_DISABLE();
  HAL_GPIO_DeInit(AS5600_KNOB_GPIO_PORT, AS5600_KNOB_SDA_PIN | AS5600_KNOB_SCL_PIN);
  g_as5600_knob.ready = 0U;
  g_as5600_knob.online = 0U;
}
