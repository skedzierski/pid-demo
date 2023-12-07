#include "tasks.h"
#include "main.h"
#include "vl6180_api.h"
#include "mpu6050.h"
#include "queue.h"
#include "semphr.h"
#include "stdio.h"

volatile extern uint8_t new_sample_ready;
extern I2C_HandleTypeDef hi2c1;
extern TaskHandle_t taskh;
void demo_tof(void* args)
{
  VL6180Dev_t dev = 0x52;
  QueueHandle_t* message_queue = args;
  taskENTER_CRITICAL();
  HAL_GPIO_WritePin(VL6180_GPIO0_GPIO_Port, VL6180_GPIO0_Pin, GPIO_PIN_SET);
  VL6180_WaitDeviceBooted(dev);
  VL6180_InitData(dev);
  VL6180_Prepare(dev);
  VL6180_RangeSetInterMeasPeriod(dev, 1000);
  VL6180_SetupGPIO1(dev, GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT, INTR_POL_HIGH);
  VL6180_RangeConfigInterrupt(dev, CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY);
  VL6180_RangeStartContinuousMode(dev);
  taskEXIT_CRITICAL();
  VL6180_RangeData_t data = {0};
  measurment m;
  m.device = VL6180;
  while(1)
  {
    xTaskNotifyWait(0xffffffff, 0, NULL, portMAX_DELAY);
    taskENTER_CRITICAL();
    VL6180_RangeGetMeasurement(dev, &data);
    m.distance = data.range_mm;
    xQueueSend(*message_queue, &m, portMAX_DELAY);
    VL6180_ClearAllInterrupt(dev);
    taskEXIT_CRITICAL();
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
  
  int16_t acc_x, gyro_x;
  QueueHandle_t* message_queue = args;
  measurment m;
  m.device = MPU6050;
  while(1)
  {
    taskENTER_CRITICAL();
    MPU6050_GetAccelerationXRAW(&acc_x);
    MPU6050_GetRotationXRAW(&gyro_x);
    m.acc_x = acc_x * acc_scale;
    m.gyro_x = gyro_x * gyro_scale;
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
    // if(uxQueueMessagesWaiting(*message_queue) == 0)
    // {
    //   vTaskSuspend(NULL);
    //   volatile eTaskState state = eTaskGetState(taskh);
    //   taskYIELD();
    // }
    xQueueReceive(*message_queue, &message_buf, 3/portTICK_PERIOD_MS);
    if(message_buf.device == VL6180)
    {
      taskENTER_CRITICAL();
      printf("distance: %ldmm\r\n", message_buf.distance);
      taskEXIT_CRITICAL();
    }
    else if(message_buf.device == MPU6050)
    {
      taskENTER_CRITICAL();
      printf("acceleration x: %f mG rotation: %f [deg/s]\r\n", message_buf.acc_x, message_buf.gyro_x);
    }
  }
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    signed char *pcTaskName )
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  while(1);
}