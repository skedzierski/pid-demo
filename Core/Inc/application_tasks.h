#pragma once

#include "cmsis_os.h"

typedef enum {
    eMPU6050,
    eVL53L0X
} source;

typedef struct {
    source device;
    union {
        struct {
            float gyro_x; 
            float acc_x;
        } vec2;
        int32_t distance;
    };
    uint32_t time_stamp;
} measurment;

void mediator_task(void* args);
void mpu_task(void *args);
void simple_logger(void* args);