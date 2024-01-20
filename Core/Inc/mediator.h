#pragma once
#include "application_tasks.h"

#include "mpu6050.h"
#include "vl53l0x_api.h"

typedef struct TOF* TOF;
typedef struct MPU6050* MPU6050;

typedef struct
{
    TOF tof;
    MPU6050 mpu;
    
} Mediator_t;

typedef Mediator_t* Mediator;
struct MPU6050
{
    Mediator m;
    MPU6050_HandleTypeDef dev;
};

void MPU_Init(MPU6050 mpu, Mediator m, MPU6050_HandleTypeDef dev);

struct TOF
{
    Mediator m;
    VL53L0X_DEV    Dev;
};

void TOF_Init(TOF t, Mediator m, VL53L0X_DEV dev);

void Mediator_notify(Mediator m, source sos);

