#pragma once

#include "mediator.h"
#include "servo.h"

struct TOF
{
    Mediator* m;
    VL53L0X_DEV    Dev;
};

struct PID
{
    Mediator* m;
    PIDController_t* pid;
};

struct SERVO
{
    Mediator* m;
    Servo_HandleTypeDef* servo;
};



void TOF_Init(TOF t, Mediator* m, VL53L0X_DEV dev);
void SERVO_Init(SERVO servo, Mediator* m, Servo_HandleTypeDef* servo_handle);
void PID_Init(PID pid_obj, Mediator* m, PIDController_t pid_ctrl);
