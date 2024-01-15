#include "pid.h"
#include "stdlib.h"
#include "string.h"

static void get_error_signal(PIDController_t* instance, float set_point, float measured_set_point)
{
    for(int i = PROCESS_SIGNAL_LEN-1; i > 0; i--)
    {
        instance->error_signal[i] = instance->error_signal[i-1];
    }
    instance->error_signal[0] = set_point - measured_set_point;
}

static float get_difference(float f_of_x, float f_of_x_minus_h)
{
    return (f_of_x - f_of_x_minus_h);
}

static float integrate(float* f, int len)
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
    get_error_signal(instance, set_point, measured_set_point);

    float new_control = instance->P*instance->error_signal[0] + 
                        instance->I*integrate(instance->error_signal, PROCESS_SIGNAL_LEN) + 
                        instance->D*get_difference(instance->error_signal[0], instance->error_signal[1]);
    return new_control;
}

int PID_Init(PIDController_t* instance, float P, float I, float D)
{
    instance->P = P;
    instance->I = I;
    instance->D = D;
    memset(instance->error_signal, 0, sizeof(instance->error_signal));
    return 0;
}

int Adapter_Init(Adapter_t* adapter, float source_max, float source_min, float dest_max, float dest_min)
{
    if(source_max < source_min)
        return -1;
    
    if(dest_max < dest_min)
        return -1;

    adapter->from_max = source_max;
    adapter->from_min = source_min;
    adapter->to_max = dest_max;
    adapter->to_min = dest_min;
    return 0;
}

float Adapter_Map(Adapter_t* adapter, float value)
{
    float mapped = (value - adapter->from_min) * (adapter->to_max - adapter->to_min) / (adapter->from_max - adapter->from_min) + adapter->to_min;
    if(mapped > adapter->to_max)
        mapped = adapter->to_max;

    if(mapped < adapter->to_min)
        mapped = adapter -> to_min;
    return mapped;
}

#if PROCESS_SIGNAL_LEN < 2
    #error "PROCESS_SIGNAL_LEN must be at least 2"
#endif
