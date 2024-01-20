#pragma once

#include "mpu6050.h"
#include "mediator.h"

struct MPU6050
{
    Mediator* m;
    MPU6050_HandleTypeDef* dev;
};

void MPU_Init(MPU6050 mpu, Mediator* m, MPU6050_HandleTypeDef* dev);
