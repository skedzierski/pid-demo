#include "mediator.h"

void Mediator_notify(Mediator m, source sos)
{
    switch(sos)
    {
        case eMPU6050:

        break;

        case eVL53L0X:

        break;
    
        default:

        break;
    }
}

void MPU_Init(MPU6050 mpu, Mediator m, MPU6050_HandleTypeDef dev)
{
    mpu->dev = dev;
    mpu->m = m;
}

void TOF_Init(TOF t, Mediator m, VL53L0X_DEV dev)
{
    t->Dev = dev;
    t->m = m;
}