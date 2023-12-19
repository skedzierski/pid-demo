/*
 * servo.c
 *
 *      Author: Karol Michalski
 *      Servo driver using STM32 timer
 */

#include "servo.h"

void SERVO_Init(Servo_HandleTypeDef *dev, TIM_HandleTypeDef *htim, uint32_t tim_base_clock, uint32_t pwm_frequency, uint32_t pwm_period, uint32_t tim_channel){
    dev->tim_handle = htim;
    dev->tim_channel = tim_channel;
    HAL_TIM_PWM_Init(dev->tim_handle);
    dev->tim_base_clock = tim_base_clock;
    dev->pwm_frequency = pwm_frequency;
    dev->pwm_period = pwm_period*10;
    SERVO_CalculateTimerSetting(dev);
    dev->tim_handle->Instance->PSC = dev->prescaler;
    dev->tim_handle->Instance->ARR = dev->counter_period;
    dev->pwm_step = ((float)dev->pwm_period / (float)dev->counter_period);
    dev->zero_pos_period = (10000/dev->pwm_step);
    SERVO_SetPosition(dev, 90.0);
    //HAL_TIM_PWM_Init(dev->tim_handle);
    HAL_TIM_PWM_Start(dev->tim_handle, dev->tim_channel);
}

void SERVO_CalculateTimerSetting(Servo_HandleTypeDef *dev){
    uint16_t i = 65535;
    uint16_t temp = 0;
    uint8_t found = 0;
    for(i=65535;i>0;i--){
        if((dev->tim_base_clock % i) == 0){
            temp = dev->tim_base_clock / i;
            if((temp % dev->pwm_frequency) == 0){
                found = 1; //TODO Do something with found var
                break;
            }
        }
    }
    dev->counter_period = i-1;
    dev->prescaler = (temp / dev->pwm_frequency)-1;
}

void SERVO_CalculatePeriod(Servo_HandleTypeDef *dev, float pos){
    float setpoint = 0;
    //if(pos <= 0) pos = 0.0; //TODO Uncomment
    // {
    //     setpoint = dev->zero_pos_period;
    //     dev->current_position = 0.0;
    //     dev->current_pulse = (uint16_t) setpoint;
    // }
    //else if(pos > 180.0) pos = 180.0; //TODO Uncomment
    pos *= 10;
    setpoint = PERIOD_PER_DEG * pos;
    setpoint /= dev->pwm_step;
    setpoint += dev->zero_pos_period;
    dev->current_pulse = (uint16_t) setpoint;
    dev->current_position = pos;
}

void SERVO_WritePosition(Servo_HandleTypeDef *dev){
    switch (dev->tim_channel)
    {
    case TIM_CHANNEL_1:
        dev->tim_handle->Instance->CCR1 = dev->current_pulse;
        break;
    case TIM_CHANNEL_2:
        dev->tim_handle->Instance->CCR2 = dev->current_pulse;
        break;
    case TIM_CHANNEL_3:
        dev->tim_handle->Instance->CCR3 = dev->current_pulse;
        break;
    case TIM_CHANNEL_4:
        dev->tim_handle->Instance->CCR4 = dev->current_pulse;
        break;
    default:
        //return SERVO_ARG_ERR; //TODO Implement returns
        break;
    }
}

void SERVO_SetPosition(Servo_HandleTypeDef *dev, float pos){
    SERVO_CalculatePeriod(dev, pos);
    SERVO_WritePosition(dev); //TODO Implement returns
}