#pragma once

#include "stdint.h"

#define PROCESS_SIGNAL_LEN 2

/**
 * @def TIME_DELTA
 * @brief time between measurments of control variable
*/
#define TIME_DELTA         0.01f

typedef struct {
    float P;
    float I;
    float D;
    float set_point[PROCESS_SIGNAL_LEN];
    float measured_set_point[PROCESS_SIGNAL_LEN];
    float error_signal[PROCESS_SIGNAL_LEN];
}PIDController_t;

typedef struct {
    float from_min, from_max, to_min, to_max;
} Adapter_t;

float Adapter_map(Adapter_t*, float);

float PID_GetNewControl(PIDController_t* instance, float measured_set_point, float set_point);
int PID_SetProportionalCoef(PIDController_t* instance, float P);
int PID_SetIntegralCoef(PIDController_t* instance, float I);
int PID_SetDiferentialCoef(PIDController_t* instance, float D);
int PID_GetInstance(PIDController_t* instance, float P, float I, float D);
