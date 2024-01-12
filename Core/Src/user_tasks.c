#include "tasks.h"
#include "main.h"
#include "vl53l0x_api.h"
#include "mpu6050.h"
#include "queue.h"
#include "semphr.h"
#include "stdio.h"

volatile extern uint8_t new_sample_ready;
extern I2C_HandleTypeDef hi2c1;
extern TaskHandle_t taskh;

// VL53L0X_Dev_t  vl53l0x_c; // center module
// VL53L0X_DEV    Dev = &vl53l0x_c;
// VL53L0X_RangingMeasurementData_t RangingData;
void demo_tof(void* args)
{
  QueueHandle_t* message_queue = args;
  //
	// VL53L0X initialisation stuff
	//
  uint32_t refSpadCount;
  uint8_t isApertureSpads;
  uint8_t VhvSettings;
  uint8_t PhaseCal;
  volatile VL53L0X_Error Status = VL53L0X_ERROR_NONE;

  Dev->I2cHandle = &hi2c1;
  Dev->I2cDevAddr = 0x52;

  //
  // VL53L0X init for Single Measurement
  //
  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(VL6180_GPIO0_GPIO_Port, VL6180_GPIO0_Pin, GPIO_PIN_SET); // Enable XSHUT
  Status += VL53L0X_DataInit( Dev );
  Status += VL53L0X_StaticInit( Dev );
  Status += VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
  Status += VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
  Status += VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING);
  Status += VL53L0X_SetInterMeasurementPeriodMilliSeconds(Dev, 1000);
  Status += VL53L0X_StartMeasurement(Dev);
  taskEXIT_CRITICAL();
  
  measurment m;
  m.device = VL6180;
  while(1)
  {
    xTaskNotifyWait(0xffffffff, 0, NULL, portMAX_DELAY);
    taskENTER_CRITICAL();
    Status += VL53L0X_GetRangingMeasurementData(Dev, &RangingData);
    Status += VL53L0X_ClearInterruptMask(Dev, 0);
    taskEXIT_CRITICAL();
    m.distance = RangingData.RangeMilliMeter;
    m.time_stamp = xTaskGetTickCount();
    xQueueSend(*message_queue, &m, portMAX_DELAY);

    vTaskResume(taskh);
  }
}

void demo_acc(void* args)
{
  int status = 0;
  float gyro_scale, acc_scale;
  status = MPU6050_Init(&hi2c1, 0);
  status += MPU6050_SetDLPF(MPU6050_DLPF_BW_20);
  status += MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
  status += MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  status += MPU6050_GetGyroScale(&gyro_scale);
  status += MPU6050_GetAccelScale(&acc_scale);

  if(status != 0)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  }
  
  int16_t acc_x, gyro_x;
  QueueHandle_t* message_queue = args;
  measurment m;
  m.device = MPU6050;
  while(1)
  {
    taskENTER_CRITICAL();
    MPU6050_GetAccelerationXRAW(&acc_x);
    MPU6050_GetRotationXRAW(&gyro_x);
    m.vec2.acc_x = acc_x * acc_scale;
    m.vec2.gyro_x = gyro_x * gyro_scale;
    m.time_stamp = xTaskGetTickCount();
    taskEXIT_CRITICAL();
    xQueueSend(*message_queue, &m, portMAX_DELAY);
    vTaskResume(taskh);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void simple_logger(void* args)
{
  measurment message_buf = {0};
  QueueHandle_t* message_queue = args;
  while(1)
  {
    if(uxQueueMessagesWaiting(*message_queue) == 0)
    {
      vTaskSuspend(NULL);
      taskYIELD();
    }
    xQueueReceive(*message_queue, &message_buf, portMAX_DELAY);
    if(message_buf.device == VL6180)
    {
      taskENTER_CRITICAL();
      printf("VL;%ld;%ld;0\r\n", message_buf.time_stamp, message_buf.distance);
      taskEXIT_CRITICAL();
    }
    else if(message_buf.device == MPU6050)
    {
      taskENTER_CRITICAL();
      printf("MPU;%ld;%f;%f\r\n", message_buf.time_stamp, message_buf.vec2.acc_x, message_buf.vec2.gyro_x);
      taskEXIT_CRITICAL();
    }
  }
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    signed char *pcTaskName )
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  while(1);
}