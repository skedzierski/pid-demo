/*
 * servo.h
 *
 *      Author: Karol Michalski
 *      Servo driver using STM32 timer
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "stm32f4xx_hal.h"

#define PERIOD_PER_DEG 5.555556         //Angle and period are multiplied by 10

typedef struct{
    TIM_HandleTypeDef *tim_handle;       /*HAL timer handler                               */
    uint32_t tim_channel;                /*Used timer channel                              */
    uint32_t tim_base_clock;             /*Timer base clock                                */
    uint16_t prescaler;                  /*Timer prescaler                                 */
    uint16_t counter_period;             /*Timer counter period                            */
    uint32_t pwm_frequency;              /*Set PWM frequency in Hz                         */
    uint32_t pwm_period;                 /*Set PWM period in us multiplied by 10           */
    uint32_t zero_pos_period;            /*CCR register value for 0deg position            */
    float pwm_step;                      /*Pulse width step                                */  
    float current_position;              /*Set servo position                              */
    uint16_t current_pulse               /*CCR register value required for current_position*/
} Servo_HandleTypeDef;

//TODO Change return type
void SERVO_Init(Servo_HandleTypeDef *dev, TIM_HandleTypeDef *htim, uint32_t tim_base_clock, uint32_t pwm_frequency, uint32_t pwm_period, uint32_t tim_channel);
void SERVO_CalculateTimerSetting(Servo_HandleTypeDef *dev);
void SERVO_SetPosition(Servo_HandleTypeDef *dev, float pos);
void SERVO_WritePosition(Servo_HandleTypeDef *dev); //TODO Change name

#endif