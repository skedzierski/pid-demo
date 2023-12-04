#include "tasks.h"
#include "main.h"
#include "vl6180_api.h"
#include "queue.h"
#include "semphr.h"
#include "stdio.h"

volatile extern uint8_t new_sample_ready;
TaskFunction_t demo(void* args)
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
  while(1)
  {
    xTaskNotifyWait(0xffffffff, 0, NULL, portMAX_DELAY);
    taskENTER_CRITICAL();
    VL6180_RangeGetMeasurement(dev, &data);
    xQueueSend(*message_queue, &data.range_mm, portMAX_DELAY);
    VL6180_ClearAllInterrupt(dev);
    taskEXIT_CRITICAL();
  }
}

TaskFunction_t simple_logger(void* args)
{
  int32_t message_buf = 0;
  QueueHandle_t* message_queue = args;
  while(1)
  {
    xQueueReceive(*message_queue, &message_buf, portMAX_DELAY);
    taskENTER_CRITICAL();
    printf("distance: %ld\r\n", message_buf);
    taskEXIT_CRITICAL();
  }
}