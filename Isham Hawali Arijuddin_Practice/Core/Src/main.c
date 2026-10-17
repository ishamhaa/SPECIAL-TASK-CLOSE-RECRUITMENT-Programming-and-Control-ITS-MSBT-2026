/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Hydrofoil Dual Servo PID
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
float angle_target = 0.0;
float pitch_flt = 0.0;

// PID
float Kp = 9.0;
float Ki = 0.1;
float Kd = 0.1;

float error, last_error = 0.0, integral = 0.0;
float pid_out;

// servo
int offset_kiri = -90;

// timing non blocking
uint32_t last_loop_time = 0;
uint32_t last_pid_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
float MPU6050_GetPitch(void);
float Compute_PID(float target, float current);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  HAL_Delay(2000);

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // servo kiri
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // servo kanan

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500 + offset_kiri );
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500 );

  MPU6050_Init();

  while (1)
  {
    uint32_t current_time = HAL_GetTick();

    // Loop jalan secara non blocking tiap 20ms
    if (current_time - last_loop_time >= 20)
    {
      last_loop_time = current_time;

      float pitch_raw = MPU6050_GetPitch();
      pitch_flt = (0.85 * pitch_flt) + (0.15 * pitch_raw);

      // hitung PID
      pid_out = Compute_PID(angle_target, pitch_flt);

      // potong outp saat dekat dengan setpoint
      if (fabsf(angle_target - pitch_flt) < 0.5)
      {
        pid_out = 0.0;
        integral = 0.0;
      }

      int pwm_val = 1500 - (int)pid_out;

      //limitter
      if (pwm_val > 2000){
        pwm_val = 2000;
      }
      if (pwm_val < 1000){
        pwm_val = 1000;
      }

      //update pwm ke servo
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_val + offset_kiri); // kiri
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_val ); //kanan
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MPU6050_Init(void)
{
  uint8_t check, data;

  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);

  if (check == 0x68)
  {
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 1000);
  }
}

// sudut
float MPU6050_GetPitch(void)
{
  uint8_t raw_buf[6];
  int16_t raw_ax, raw_ay, raw_az;
  float ax, ay, az;

  // baca reg akselerometer
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, raw_buf, 6, 1000);

  raw_ax = (int16_t)(raw_buf[0] << 8 | raw_buf[1]);
  raw_ay = (int16_t)(raw_buf[2] << 8 | raw_buf[3]);
  raw_az = (int16_t)(raw_buf[4] << 8 | raw_buf[5]);

  ax = raw_ax / 16384.0;
  ay = raw_ay / 16384.0;
  az = raw_az / 16384.0;

  //sudut pitch ke derajat
  float pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0 / M_PI);
  return pitch_deg;
}

//hitung PID
float Compute_PID(float target, float current)
{
  uint32_t now = HAL_GetTick();
  float dt = (now - last_pid_time) / 1000.0f;

  //proteksi jika loop berjalan terlalu cepat
  if (dt <= 0.0f) {
      dt = 0.02f;
  }

  last_pid_time = now;

  error = target - current;

  float p_val = Kp * error;

  integral += error * dt;
  float i_val = Ki * integral;

  float d_val = Kd * ((error - last_error) / dt);

  last_error = error;

  return p_val + i_val + d_val;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
