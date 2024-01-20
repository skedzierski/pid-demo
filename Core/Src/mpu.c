#include "mpu.h"

void MPU6050_Init(MPU* mpu, Mediator* m, MPU6050_HandleTypeDef dev)
{
    mpu->m = m;
    mpu->dev = dev;
}