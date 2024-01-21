#include "application_tasks.h"
#include "main.h"
#include "vl53l0x_api.h"
#include "mpu6050.h"
#include "queue.h"
#include "stdio.h"
#include "pid.h"
#include "servo.h"
#include "tim.h"
#include "stream_buffer.h"
#include "mediator.h"

extern I2C_HandleTypeDef hi2c1;
extern TaskHandle_t taskh;

void mediator_task(void* args)
{
    Servo_HandleTypeDef servo_dev;

    VL53L0X_Dev_t vl53l0x_c; // center module
    VL53L0X_DEV Dev = &vl53l0x_c;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    volatile VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    QueueHandle_t *message_queue = (QueueHandle_t*)args;

    PIDController_t pid;

    Adapter_t adp;

    SERVO_Init(&servo_dev, &htim3, 84e6, 100, 10000, TIM_CHANNEL_1);
    SERVO_SetPosition(&servo_dev, 90.0);
    //TOF Init Start
    HAL_NVIC_DisableIRQ(TOF_IRQ_EXTI_IRQn);
    HAL_GPIO_WritePin(TOF_SHUT_GPIO_Port, TOF_SHUT_Pin, GPIO_PIN_RESET); // Enable XSHUT
    HAL_Delay(20);
    HAL_GPIO_WritePin(TOF_SHUT_GPIO_Port, TOF_SHUT_Pin, GPIO_PIN_SET); // Enable XSHUT
    HAL_Delay(20);

    Dev->I2cHandle = &hi2c1;
    Dev->I2cDevAddr = 0x52;
    Status = VL53L0X_DataInit(Dev);
    Status = VL53L0X_StaticInit(Dev);
    Status = VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
    Status = VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
    Status = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

    // Enable/Disable Sigma and Signal check
    Status = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    Status = VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    Status = VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1 * 65536));
    Status = VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536));
    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
    Status = VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    Status = VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    Status = VL53L0X_ClearInterruptMask(Dev, 0);
    //TOF Init End

    if (Status != 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        while(1);
    }

    PID_Init(&pid, 13, 0, 100);

    Adapter_Init(&adp, 10000.f, -10000.f, 120.f, 60.f);

    MediatorBuilder builder;
    MediatorBuilder_AddPID(&builder, &pid);
    MediatorBuilder_AddServo(&builder, &servo_dev);
    MediatorBuilder_AddTOF(&builder, Dev);
    MediatorBuilder_AddAdapter(&builder, &adp);
    Mediator m = MediatorBuilder_GetMediator(&builder);
    measurment meas;
    meas.device = eVL53L0X;
    while(1)
    {
        Mediator_BalanceBallAt(&m, 250);
        meas.distance = Mediator_GetRange(&m);
        xQueueSend(*message_queue, &meas, portMAX_DELAY);
        vTaskResume(taskh);
    }
}

void mpu_task(void *args)
{
    MPU6050_HandleTypeDef mpu6050_dev;
    volatile MPU6050_StatusTypeDef status = 0; //TODO delete volatile
    float gyro_scale, acc_scale;
    int16_t gyro_x_offset, gyro_y_offset, gyro_z_offset;
    int16_t acc_x, gyro_x;
    QueueHandle_t *message_queue = args;
    measurment m;
    m.device = eMPU6050;

    status = MPU6050_Init(&mpu6050_dev, &hi2c1, 0, HAL_I2C_ERROR_TIMEOUT);
    status += MPU6050_SetDLPF(&mpu6050_dev, MPU6050_A_DLPF_BW_20);
    status += MPU6050_SetFullScaleGyroRange(&mpu6050_dev, MPU6050_GYRO_FS_250);
    status += MPU6050_SetFullScaleAccelRange(&mpu6050_dev, MPU6050_ACCEL_FS_2);
    status += MPU6050_GetGyroScale(&mpu6050_dev, &gyro_scale);
    status += MPU6050_GetAccelScale(&mpu6050_dev, &acc_scale);

    status = MPU6050_MeasureGyroOffsetX(&mpu6050_dev, &gyro_x_offset);
    status = MPU6050_SetGyroOffsetX(&mpu6050_dev, gyro_x_offset);
    status = MPU6050_MeasureGyroOffsetY(&mpu6050_dev, &gyro_y_offset);
    status = MPU6050_SetGyroOffsetY(&mpu6050_dev, gyro_y_offset);
    status = MPU6050_MeasureGyroOffsetZ(&mpu6050_dev, &gyro_z_offset);
    status = MPU6050_SetGyroOffsetZ(&mpu6050_dev, gyro_z_offset);

    status += MPU6050_SetSampleRateDiv(&mpu6050_dev, 255);
    status += MPU6050_SetIntPinActiveLevel(&mpu6050_dev, MPU6050_INTLVL_ACTIVELOW);
    status += MPU6050_SetIntPinMode(&mpu6050_dev, MPU6050_INTDRV_OPENDRAIN);
    status += MPU6050_SetIntPinLatch(&mpu6050_dev, MPU6050_INTLATCH_50USPULSE);
    status += MPU6050_SetIntPinClearMode(&mpu6050_dev, MPU6050_INTCLEAR_ANYREAD);
    status += MPU6050_EnableRawReadyInt(&mpu6050_dev, MPU6050_INTRAWREADY_ENABLE);

    if (status != 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    }

    while (1)
    {
        xTaskNotifyWait(0xffffffff, 0, NULL, portMAX_DELAY);
        status = 0;
        taskENTER_CRITICAL();
        status = MPU6050_GetAccelerationXRAW(&mpu6050_dev, &acc_x);
        if(status)
        {
            //TODO handle err
        }
        status = MPU6050_GetRotationXRAW(&mpu6050_dev, &gyro_x);
        if(status)
        {
            //TODO handle err
        }
        taskEXIT_CRITICAL();
        m.vec2.acc_x = acc_x * acc_scale;
        m.vec2.gyro_x = gyro_x * gyro_scale;
        m.time_stamp = xTaskGetTickCount();

        xQueueSend(*message_queue, &m, portMAX_DELAY);
        vTaskResume(taskh);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void simple_logger(void *args)
{
    measurment message_buf = {0};
    QueueHandle_t *message_queue = args;
    while (1)
    {
        if (uxQueueMessagesWaiting(*message_queue) == 0)
        {
            vTaskSuspend(NULL);
            taskYIELD();
        }
        xQueueReceive(*message_queue, &message_buf, portMAX_DELAY);
        if (message_buf.device == eVL53L0X)
        {
            taskENTER_CRITICAL();
            printf("VL;%ld;%ld;0\r\n", message_buf.time_stamp, message_buf.distance);
            taskEXIT_CRITICAL();
        }
        else if (message_buf.device == eMPU6050)
        {
            taskENTER_CRITICAL();
            printf("MPU;%ld;%f;%f\r\n", message_buf.time_stamp, message_buf.vec2.acc_x, message_buf.vec2.gyro_x);
            taskEXIT_CRITICAL();
        }
    }
}
