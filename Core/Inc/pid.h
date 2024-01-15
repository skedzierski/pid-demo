#pragma once

#include "stdint.h"

#define PROCESS_SIGNAL_LEN 2

typedef struct {
    float P;
    float I;
    float D;
    float error_signal[PROCESS_SIGNAL_LEN];
}PIDController_t;

typedef struct {
    float from_min, from_max, to_min, to_max;
} Adapter_t;

int Adapter_Init(Adapter_t* adapter, float source_max, float source_min, float dest_max, float dest_min);
float Adapter_Map(Adapter_t*, float);

float PID_GetNewControl(PIDController_t* instance, float measured_set_point, float set_point);
int PID_SetProportionalCoef(PIDController_t* instance, float P);
int PID_SetIntegralCoef(PIDController_t* instance, float I);
int PID_SetDiferentialCoef(PIDController_t* instance, float D);
int PID_Init(PIDController_t* instance, float P, float I, float D);
