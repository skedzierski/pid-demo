/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_HandleTypeDef mpu6050_dev;

uint8_t test = 123;
uint8_t whoami = 0;

float gyro_scale = 0;
int16_t gyro_x_offset = 0;

int16_t gyro_x_raw = 0;
float gyro_x_scaled = 0;

int16_t gyro_y_raw = 0;
float gyro_y_scaled = 0;

int16_t gyro_z_raw = 0;
float gyro_z_scaled = 0;

float accel_scale = 0;

int16_t accel_x_raw = 0;
float accel_x_scaled = 0;

int16_t accel_y_raw = 0;
float accel_y_scaled = 0;

int16_t accel_z_raw = 0;
float accel_z_scaled = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  test = MPU6050_Init(&mpu6050_dev, &hi2c1, 0, 30);
  test = MPU6050_SetDLPF(&mpu6050_dev, MPU6050_DLPF_BW_20);
  test = MPU6050_SetADLPF(&mpu6050_dev, MPU6050_A_DLPF_BW_20);
  test = MPU6050_SetFullScaleGyroRange(&mpu6050_dev, MPU6050_GYRO_FS_2000);
  test = MPU6050_SetFullScaleAccelRange(&mpu6050_dev, MPU6050_ACCEL_FS_2);
  test = MPU6050_GetDeviceID(&mpu6050_dev, &whoami);
  test = MPU6050_GetGyroScale(&mpu6050_dev, &gyro_scale);
  test = MPU6050_GetAccelScale(&mpu6050_dev, &accel_scale);

  test = MPU6050_GetRotationXRAW(&mpu6050_dev, &gyro_x_raw);
  test = MPU6050_MeasureGyroOffsetX(&mpu6050_dev, &gyro_x_offset);
  test = MPU6050_SetGyroOffsetX(&mpu6050_dev, gyro_x_offset);

  // test = MPU6050_SetSampleRateDiv(&mpu6050_dev, 255);
  // test = MPU6050_SetIntPinActiveLevel(&mpu6050_dev, MPU6050_INTLVL_ACTIVELOW);
  // test = MPU6050_SetIntPinMode(&mpu6050_dev, MPU6050_INTDRV_OPENDRAIN);
  // test = MPU6050_SetIntPinLatch(&mpu6050_dev, MPU6050_INTLATCH_50USPULSE);
  // test = MPU6050_SetIntPinClearMode(&mpu6050_dev, MPU6050_INTCLEAR_ANYREAD);
  // test = MPU6050_EnableRawReadyInt(&mpu6050_dev, MPU6050_INTRAWREADY_ENABLE);

 

  /* USER CODE END 2 */

  /* Init scheduler */
  //osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  //MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
   
    test = MPU6050_GetRotationXRAW(&mpu6050_dev, &gyro_x_raw);
    gyro_x_scaled = (float)gyro_x_raw * gyro_scale;
    test = MPU6050_GetRotationYRAW(&mpu6050_dev, &gyro_y_raw);
    gyro_y_scaled = (float)gyro_y_raw * gyro_scale;
    test = MPU6050_GetRotationZRAW(&mpu6050_dev, &gyro_z_raw);
    gyro_z_scaled = (float)gyro_z_raw * gyro_scale;

    test = MPU6050_GetAccelerationXRAW(&mpu6050_dev, &accel_x_raw);
    accel_x_scaled = (float)accel_x_raw * accel_scale;
    test = MPU6050_GetAccelerationYRAW(&mpu6050_dev, &accel_y_raw);
    accel_y_scaled = (float)accel_y_raw * accel_scale;
    test = MPU6050_GetAccelerationZRAW(&mpu6050_dev, &accel_z_raw);
    accel_z_scaled = (float)accel_z_raw * accel_scale;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
