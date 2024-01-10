#include "pid.h"
#include "stdlib.h"
#include "string.h"

static void array_rshift(PIDController_t* instance)
{
    for(int i = PROCESS_SIGNAL_LEN-1; i > 0; i--)
    {
        instance->set_point[i] = instance->set_point[i-1];
        instance->measured_set_point[i] = instance->measured_set_point[i-1];
    }
}

static void get_error_signal(PIDController_t* instance)
{
    for(int i = 0; i < PROCESS_SIGNAL_LEN; i++)
    {
        instance->error_signal[i] = instance->set_point[i] - instance->measured_set_point[i];
    }
    
}

//static float h = TIME_DELTA;
static float get_difference(float f_of_x, float f_of_x_minus_h)
{
    return (f_of_x_minus_h - f_of_x);
}

static float dt = TIME_DELTA;
static float trapez(float* f, int len)
{
    float integral = 0;
    for(int i = 0; i < len; i++)
    {
        integral += f[i]; 
    }
    return integral;
}

float PID_GetNewControl(PIDController_t* instance, float measured_set_point, float set_point)
{
    array_rshift(instance);
    instance->measured_set_point[0] = measured_set_point;
    instance->set_point[0] = set_point;
    get_error_signal(instance);

    float new_control = instance->P*instance->error_signal[0] + 
                        instance->I*trapez(instance->error_signal, PROCESS_SIGNAL_LEN) + 
                        instance->D*get_difference(instance->error_signal[0], instance->error_signal[1]);
    return new_control;
}

int PID_GetInstance(PIDController_t* instance, float P, float I, float D)
{
    instance->P = P;
    instance->I = I;
    instance->D = D;
    memset(instance->error_signal, 0, sizeof(instance->error_signal));
    memset(instance->set_point, 0, sizeof(instance->error_signal));
    memset(instance->measured_set_point, 0, sizeof(instance->error_signal));
    return 0;
}

float Adapter_map(Adapter_t* adapter, float value)
{
    // float slope = (adapter->to_max - adapter->to_min) / (adapter->from_max - adapter->from_min);
    // return (value - adapter->to_min)*slope + adapter->to_min;

    return (value - adapter->from_min) * (adapter->to_max - adapter->to_min) / (adapter->from_max - adapter->from_min) + adapter->to_min;
}

#if PROCESS_SIGNAL_LEN < 2
    #error "PROCESS_SIGNAL_LEN must be at least 2"
#endif
