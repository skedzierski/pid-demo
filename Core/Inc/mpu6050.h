#ifndef MPU6050_H_
#define MPU6050_H_

#include "mpu6050_regmap.h"
#include "stm32f4xx_hal.h"

typedef struct{
    I2C_HandleTypeDef *i2c_handle;       /*HAL I2C Handler                              */
    uint8_t i2c_addres;                  /*0 = 0xD0 (AD0 low) 1 = 0xD1 (AD0 high)       */
    uint8_t gyro_range;                  /*Currently set gyroscope full scale range     */
    float gyro_scale;                    /*Gyroscope scaler for current full range      */
    uint8_t accel_range;                 /*Currently set accelerometer full scale range */
    float accel_scale;                   /*Accelerometer scaler for current full range  */     
} MPU6050_HandleTypeDef;

typedef enum 
{
  MPU6050_OK       = 0x00U,
  MPU6050_I2C_ERR  = 0x01U,
  MPU6050_ARG_ERR  = 0x02U,
} MPU6050_StatusTypeDef;

MPU6050_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c, uint8_t address_select);
MPU6050_StatusTypeDef MPU6050_GetDeviceID(uint8_t *data);
MPU6050_StatusTypeDef MPU6050_DeviceReset(void);
MPU6050_StatusTypeDef MPU6050_SetSleepMode(uint8_t mode);
MPU6050_StatusTypeDef MPU6050_SetClockSource(uint8_t clk_source);
MPU6050_StatusTypeDef MPU6050_SetDLPF(uint8_t filter_value);
MPU6050_StatusTypeDef MPU6050_SetFullScaleGyroRange(uint8_t gyro_range);
MPU6050_StatusTypeDef MPU6050_SetFullScaleAccelRange(uint8_t accel_range);
MPU6050_StatusTypeDef MPU6050_GetGyroScale(float *gyro_scale);
MPU6050_StatusTypeDef MPU6050_GetRotationXRAW(int16_t *gyro_x);
MPU6050_StatusTypeDef MPU6050_GetRotationYRAW(int16_t *gyro_y);
MPU6050_StatusTypeDef MPU6050_GetRotationZRAW(int16_t *gyro_z);
MPU6050_StatusTypeDef MPU6050_GetAccelScale(float *accel_scale);
MPU6050_StatusTypeDef MPU6050_GetAccelerationXRAW(int16_t *accel_x);
MPU6050_StatusTypeDef MPU6050_GetAccelerationYRAW(int16_t *accel_y);
MPU6050_StatusTypeDef MPU6050_GetAccelerationZRAW(int16_t *accel_z);

#endif