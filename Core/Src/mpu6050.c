/*
 * mpu6050.c
 *
 *      Author: Karol Michalski
 *      MPU-6050 Accelerometer and gyroscoper driver
 */

#include "mpu6050.h"

//I2C_HandleTypeDef *i2c; //test
#define I2C_TIMEOUT 10 //test

MPU6050_HandleTypeDef mpu6050_handle;

/**
  * @brief  Initialise MPU6050 device
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure
  * @param  address_select Select AD0 pin state on MP6050: 0 = 0xD0 (AD0 low), 1 = 0xD1 (AD0 high)
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c, uint8_t address_select)
{
    MPU6050_StatusTypeDef ret;
	mpu6050_handle.i2c_handle = hi2c;
    if(address_select == 0) mpu6050_handle.i2c_addres = MPU6050_ADDRESS_LOW;
    else if(address_select == 1) mpu6050_handle.i2c_addres = MPU6050_ADDRESS_HIGH;
    else return MPU6050_ARG_ERR;
	ret = MPU6050_DeviceReset();
    if(ret) return ret;
    //MPU6050_SetSleepEnabled(0);
    //MPU6050_SetClockSource(MPU6050_CLOCK_INTERNAL);
    //MPU6050_SetDlpf(MPU6050_DLPF_BW_20);
    //MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
    //MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    return MPU6050_OK;
}

/**
  * @brief  Get MPU6050 DeviceID from WHO_AM_I register (DeviceID is the currently set I2C address)
  * @param  data Pointer to uint8_t variable in which read data will be saved
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_GetDeviceID(uint8_t *data)
{
	if(HAL_I2C_Mem_Read(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_WHO_AM_I, 1, data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR; // TODO Change ADDRES_LOW according to i2c_addres_setting
	*data = *data<<1;
    return MPU6050_OK;
}

MPU6050_StatusTypeDef MPU6050_DeviceReset(void){
    uint8_t data;
	if(HAL_I2C_Mem_Read(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR;
	//data &= ~(1<<MPU6050_PWR1_DEVICE_RESET_BIT);
	data |= (1 << MPU6050_PWR1_DEVICE_RESET_BIT);
	if(HAL_I2C_Mem_Write(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR;
    return MPU6050_OK;
}


