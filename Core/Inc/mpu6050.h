#ifndef MPU6050_H_
#define MPU6050_H_

#include "mpu6050_regmap.h"
#include "stm32f4xx_hal.h"

typedef struct{
    I2C_HandleTypeDef *i2c_handle;       //HAL I2C Handler
    uint8_t i2c_addres        //0 = 0xD0 (AD0 low) 1 = 0xD1 (AD0 high)
    

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


#endif