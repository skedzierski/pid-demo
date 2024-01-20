#pragma once
#include "FreeRTOS.h"
#include "mpu6050.h"
#include "vl53l0x_api.h"
#include "queue.h"
#include "pid.h"
#include "servo.h"

typedef struct TOF* TOF;
typedef struct MPU6050* MPU6050;

typedef enum
{
    eMPU,
    eTOF,
    eSERVO,
    ePID
} Component;

typedef struct
{
    VL53L0X_DEV tof;
    MPU6050 mpu;
    PIDController_t* pid;
    Adapter_t* adapter;
    Servo_HandleTypeDef* servo;
    VL53L0X_RangingMeasurementData_t data;
    
} Mediator;

void Mediator_Init(Mediator* m, MPU6050_HandleTypeDef* mpu6050_dev, VL53L0X_DEV tof_dev);
void Mediator_BalanceBallAt(Mediator* m, uint16_t pos);

typedef struct
{
    Mediator m;
} MediatorBuilder;

void MediatorBuilder_AddPID(MediatorBuilder* builder, PIDController_t* pid);
void MediatorBuilder_AddServo(MediatorBuilder* builder, Servo_HandleTypeDef* servo);
void MediatorBuilder_AddTOF(MediatorBuilder* builder, VL53L0X_DEV tof);
void MediatorBuilder_AddAdapter(MediatorBuilder* builder, Adapter_t* adpt);
Mediator MediatorBuilder_GetMediator(MediatorBuilder* builder);
