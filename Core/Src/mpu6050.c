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
    MPU6050_SetSleepMode(0);
    MPU6050_SetClockSource(MPU6050_CLOCK_INTERNAL);
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

/**
  * @brief  Reset MPU6050 registers to deafult values
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_DeviceReset(void){
    uint8_t data;
	if(HAL_I2C_Mem_Read(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR;
	//data &= ~(1<<MPU6050_PWR1_DEVICE_RESET_BIT);
	data |= (1 << MPU6050_PWR1_DEVICE_RESET_BIT);
	if(HAL_I2C_Mem_Write(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR;
    return MPU6050_OK;
}

/**
  * @brief  Toggle MPU6050 sleep mode
  * @param mode Select sleep mode state: 1 - enabled 0 - disabled
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetSleepMode(uint8_t mode)
{
	uint8_t data;
	if(HAL_I2C_Mem_Read(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR;
	//data &= ~(1<<MPU6050_PWR1_SLEEP_BIT);
	data |= ((mode & 0b00000001) << MPU6050_PWR1_SLEEP_BIT);
	if(HAL_I2C_Mem_Write(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR;
    return MPU6050_OK;
}

/**
  * @brief  Select MPU6050 clock source
  * @param clk_source Use MPU6050_CLOCK_INTERNAL for internal 20MHz oscillator
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetClockSource(uint8_t clk_source)
{
	uint8_t data;
	if(HAL_I2C_Mem_Read(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR;
	data &= 0b11111000;
	data |= (clk_source & 0b00000111);
	if(HAL_I2C_Mem_Write(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR;
    return MPU6050_OK;
}

/**
  * @brief  Configure MPU6050 Data Low Pass Filter. This filter only applies for gyroscope and temperature sensor.
  * @param clk_source Use MPU6050_DLPF_BW_* defined values to select filter bandiwdth.
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetDLPF(uint8_t filter_value)
{
	uint8_t data;
	if(HAL_I2C_Mem_Read(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR;
	data &= 0b11111000;
	data |= (filter_value & 0b00000111);
	if(HAL_I2C_Mem_Write(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR;
    return MPU6050_OK;
}

MPU6050_StatusTypeDef MPU6050_SetFullScaleGyroRange(uint8_t gyro_range)
{
	uint8_t data;
	if(HAL_I2C_Mem_Read(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR;
	data &= 0b11100111;
	data |= ((gyro_range & 0b00000011) << MPU6050_GCONFIG_FS_SEL_BIT);
	if(HAL_I2C_Mem_Write(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR;

	switch(gyro_range)
	{
		case MPU6050_GYRO_FS_250:
			mpu6050_handle.gyro_scale = 0.0076294;
			break;
		case MPU6050_GYRO_FS_500:
			mpu6050_handle.gyro_scale = 0.0152588;
			break;
		case MPU6050_GYRO_FS_1000:
			mpu6050_handle.gyro_scale = 0.0305176;
			break;
		case MPU6050_GYRO_FS_2000:
			mpu6050_handle.gyro_scale = 0.0610352;
			break;
		default:
            return MPU6050_ARG_ERR;
			break;
	}
    return MPU6050_OK;
}

MPU6050_StatusTypeDef MPU6050_SetFullScaleAccelRange(uint8_t accel_range)
{
	uint8_t data;
	if(HAL_I2C_Mem_Read(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR;
	data &= 0xE7;
	data |= ((accel_range & 0x7) << 3);
	if(HAL_I2C_Mem_Write(mpu6050_handle.i2c_handle, mpu6050_handle.i2c_addres, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, I2C_TIMEOUT)) return MPU6050_I2C_ERR;

	switch(accel_range)
	{
		case MPU6050_ACCEL_FS_2:
			mpu6050_handle.accel_scale = 0.0610352;
			break;
		case MPU6050_ACCEL_FS_4:
			mpu6050_handle.accel_scale = 0.1220703;
			break;
		case MPU6050_ACCEL_FS_8:
			mpu6050_handle.accel_scale = 0.2441406;
			break;
		case MPU6050_ACCEL_FS_16:
			mpu6050_handle.accel_scale = 0.4882813;
			break;
		default:
            return MPU6050_ARG_ERR;
			break;
	}
    return MPU6050_OK;
}