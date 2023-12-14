/*
 * mpu6050.c
 *
 *      Author: Karol Michalski
 *      MPU-6050 Accelerometer and gyroscoper driver
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "mpu6050_regmap.h"
#include "stm32f4xx_hal.h"

#define MPU6050_GYRO_OFFSET_SAMPLES 100  /*Samples to take for averaging gyro offset*/

typedef struct{
    I2C_HandleTypeDef *i2c_handle;       /*HAL I2C Handler                              */
    uint8_t i2c_address;                 /*0 = 0xD0 (AD0 low) 1 = 0xD1 (AD0 high)       */
    uint32_t i2c_timeout;                /*HAL I2C Timeout                              */
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
  MPU6050_TIMEOUT  = 0x03U,
} MPU6050_StatusTypeDef;

MPU6050_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c, uint8_t address_select, uint32_t i2c_timeout);
MPU6050_StatusTypeDef MPU6050_GetDeviceID(MPU6050_HandleTypeDef *dev, uint8_t *data);
MPU6050_StatusTypeDef MPU6050_DeviceReset(MPU6050_HandleTypeDef *dev);
MPU6050_StatusTypeDef MPU6050_WaitForReset(MPU6050_HandleTypeDef *dev, uint32_t timeout);
MPU6050_StatusTypeDef MPU6050_SetSleepMode(MPU6050_HandleTypeDef *dev, uint8_t mode);
MPU6050_StatusTypeDef MPU6050_SetClockSource(MPU6050_HandleTypeDef *dev, uint8_t clk_source);
MPU6050_StatusTypeDef MPU6050_SetDLPF(MPU6050_HandleTypeDef *dev, uint8_t filter_value);
MPU6050_StatusTypeDef MPU6050_SetADLPF(MPU6050_HandleTypeDef *dev, uint8_t filter_value);
MPU6050_StatusTypeDef MPU6050_SetFullScaleGyroRange(MPU6050_HandleTypeDef *dev, uint8_t gyro_range);
MPU6050_StatusTypeDef MPU6050_SetFullScaleAccelRange(MPU6050_HandleTypeDef *dev, uint8_t accel_range);
MPU6050_StatusTypeDef MPU6050_GetGyroScale(MPU6050_HandleTypeDef *dev, float *gyro_scale);
MPU6050_StatusTypeDef MPU6050_GetRotationXRAW(MPU6050_HandleTypeDef *dev, int16_t *gyro_x);
MPU6050_StatusTypeDef MPU6050_GetRotationYRAW(MPU6050_HandleTypeDef *dev, int16_t *gyro_y);
MPU6050_StatusTypeDef MPU6050_GetRotationZRAW(MPU6050_HandleTypeDef *dev, int16_t *gyro_z);
MPU6050_StatusTypeDef MPU6050_GetAccelScale(MPU6050_HandleTypeDef *dev, float *accel_scale);
MPU6050_StatusTypeDef MPU6050_GetAccelerationXRAW(MPU6050_HandleTypeDef *dev, int16_t *accel_x);
MPU6050_StatusTypeDef MPU6050_GetAccelerationYRAW(MPU6050_HandleTypeDef *dev, int16_t *accel_y);
MPU6050_StatusTypeDef MPU6050_GetAccelerationZRAW(MPU6050_HandleTypeDef *dev, int16_t *accel_z);
MPU6050_StatusTypeDef MPU6050_SetSampleRateDiv(MPU6050_HandleTypeDef *dev, uint8_t div);
MPU6050_StatusTypeDef MPU6050_SetIntPinActiveLevel(MPU6050_HandleTypeDef *dev, uint8_t level);
MPU6050_StatusTypeDef MPU6050_SetIntPinMode(MPU6050_HandleTypeDef *dev, uint8_t mode);
MPU6050_StatusTypeDef MPU6050_SetIntPinLatch(MPU6050_HandleTypeDef *dev, uint8_t mode);
MPU6050_StatusTypeDef MPU6050_SetIntPinClearMode(MPU6050_HandleTypeDef *dev, uint8_t mode);
MPU6050_StatusTypeDef MPU6050_EnableRawReadyInt(MPU6050_HandleTypeDef *dev, uint8_t mode);
//int16_t MPU6050_CalculateAverage(const int16_t *data, uint32_t samples);
MPU6050_StatusTypeDef MPU6050_MeasureGyroOffsetX(MPU6050_HandleTypeDef *dev, int16_t *offset);
MPU6050_StatusTypeDef MPU6050_MeasureGyroOffsetY(MPU6050_HandleTypeDef *dev, int16_t *offset);
MPU6050_StatusTypeDef MPU6050_MeasureGyroOffsetZ(MPU6050_HandleTypeDef *dev, int16_t *offset);
MPU6050_StatusTypeDef MPU6050_SetGyroOffsetX(MPU6050_HandleTypeDef *dev, int16_t offset);

#endif