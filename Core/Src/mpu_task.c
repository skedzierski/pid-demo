#include "mpu_task.h"

void MPU_Init(MPU6050 mpu, Mediator* m, MPU6050_HandleTypeDef* dev)
{
    mpu->m = m;
    mpu->dev = dev;
}

void get_mpu_data_task(void* args)
{
    MPU6050 mpu;
    MPU6050_StatusTypeDef status = 0;
    int16_t gyro_x_raw, acc_x_raw;
    while(1)
    {
        xTaskNotifyWait();
        status += MPU6050_GetAccelerationXRAW(m->mpu, &acc_x_raw);
        status += MPU6050_GetRotationXRAW(m->mpu, &gyro_x_raw);
        status += MPU6050_GetAccelScale(m->mpu, &gyro_x_scale);
        status += MPU6050_GetGyroScale(m->mpu, &gyro_x_scale);
        Mediator_notify();
    }
}

void mpu_data_ready_task(void* args)
{
    //MPU6050 mpu;
    //MPU6050_StatusTypeDef status = 0;
    while(1)
    {
        xTaskNotifyFromISR();
        Mediator_notify();
    }
}

void mpu_prepare_data_task(void* args)
{   
    MPU6050 mpu;
    MPU6050_StatusTypeDef status = 0;
    int16_t gyro_x_scale, acc_x_scale;
    measurment m;
    m.device = eMPU6050;
    while(1)
    {
        xTaskNotifyWait();
        m.vec2.acc_x = acc_x * acc_x_scale;
        m.vec2.gyro_x = gyro_x * gyro_x_scale;
        m.time_stamp = xTaskGetTickCount();
        Mediator_notify();
    }
}

