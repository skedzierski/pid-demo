/*
 * mpu6050.c
 *
 *      Author: Karol Michalski
 *      MPU-6050 Accelerometer and gyroscoper driver
 */

#include "mpu6050.h"


/**
  * @brief  Initialise MPU6050 device
  * @param  dev MPU6050 Handler
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure
  * @param  address_select Select AD0 pin state on MP6050: 0 = 0xD0 (AD0 low), 1 = 0xD1 (AD0 high)
  * @param  i2c_timeout HAL I2C timeout duration
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_Init(MPU6050_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c, uint8_t address_select, uint32_t i2c_timeout)
{
    MPU6050_StatusTypeDef ret;
	  dev->i2c_handle = hi2c;
    dev->i2c_timeout = i2c_timeout;
    if(address_select == 0) dev->i2c_address = MPU6050_ADDRESS_LOW;
    else if(address_select == 1) dev->i2c_address = MPU6050_ADDRESS_HIGH;
    else return MPU6050_ARG_ERR;
	  ret = MPU6050_DeviceReset(dev);
    if(ret) return ret;
    HAL_Delay(100); //TODO Delete
    MPU6050_WaitForReset(dev, 1000);
    MPU6050_SetSleepMode(dev, 0);
    MPU6050_SetClockSource(dev, MPU6050_CLOCK_INTERNAL);
    //MPU6050_SetDlpf(MPU6050_DLPF_BW_20);
    //MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
    //MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    return MPU6050_OK;
}

/**
  * @brief  Get MPU6050 DeviceID from WHO_AM_I register (DeviceID is the currently set I2C address)
  * @param  dev MPU6050 Handler
  * @param  data Pointer to uint8_t variable in which read data will be saved
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_GetDeviceID(MPU6050_HandleTypeDef *dev, uint8_t *data)
{
	if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_WHO_AM_I, 1, data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
	*data = *data<<1;
  return MPU6050_OK;
}

/**
  * @brief  Reset MPU6050 registers to deafult values
  * @param  dev MPU6050 Handler
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_DeviceReset(MPU6050_HandleTypeDef *dev){
  uint8_t data;
	if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
	//data &= ~(1<<MPU6050_PWR1_DEVICE_RESET_BIT);
	data |= (1 << MPU6050_PWR1_DEVICE_RESET_BIT);
	if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  return MPU6050_OK;
}

/**
  * @brief  Checks DEVICE_RESET bit untill it clears.
  * @param  dev MPU6050 Handler
  * @param  timeout Time in SysTicks untill functions returns if DEVICE_RESET does not reset
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_WaitForReset(MPU6050_HandleTypeDef *dev, uint32_t timeout){
  uint8_t data;
  uint32_t time_start;
  time_start = HAL_GetTick();
  do{
  if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  data &= 0b10000000;
  if((time_start - HAL_GetTick()) > timeout)  return MPU6050_TIMEOUT;
  }while(data != 0);
  return MPU6050_OK;
}

/**
  * @brief  Toggle MPU6050 sleep mode
  * @param  dev MPU6050 Handler
  * @param  mode Select sleep mode state: 1 - enabled 0 - disabled
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetSleepMode(MPU6050_HandleTypeDef *dev, uint8_t mode)
{
	uint8_t data;
	if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
	data &= ~(1<<MPU6050_PWR1_SLEEP_BIT);
	data |= ((mode & 0b00000001) << MPU6050_PWR1_SLEEP_BIT);
	if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  return MPU6050_OK;
}//TODO Add ARG_ERR check

/**
  * @brief  Select MPU6050 clock source
  * @param  dev MPU6050 Handler
  * @param  clk_source Use MPU6050_CLOCK_INTERNAL for internal 20MHz oscillator
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetClockSource(MPU6050_HandleTypeDef *dev, uint8_t clk_source)
{
	uint8_t data;
	if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
	data &= 0b11111000;
	data |= (clk_source & 0b00000111);
	if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  return MPU6050_OK;
}//TODO Add ARG_ERR check

/**
  * @brief  Configure MPU6050 Data Low Pass Filter. This filter only applies for gyroscope and temperature sensor.
  * @param  dev MPU6050 Handler
  * @param  filter_value Use MPU6050_DLPF_BW_* defined values to select filter bandiwdth.
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetDLPF(MPU6050_HandleTypeDef *dev, uint8_t filter_value)
{
	uint8_t data;
	if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_CONFIG, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
	data &= 0b11111000;
	data |= (filter_value & 0b00000111);
	if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, MPU6050_RA_CONFIG, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  return MPU6050_OK;
}//TODO Add ARG_ERR check

/**
  * @brief  Configure MPU6050 Accelerometer Data Low Pass Filter. This filter only applies for accelerometer.
  * @param  dev MPU6050 Handler
  * @param  filter_value Use MPU6050_A_DLPF_BW_* defined values to select filter bandiwdth.
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetADLPF(MPU6050_HandleTypeDef *dev, uint8_t filter_value)
{
	uint8_t data;
	if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_ACCEL_CONFIG_2, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
	data &= 0b11111000;
	data |= (filter_value & 0b00000111);
	if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, MPU6050_RA_ACCEL_CONFIG_2, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  return MPU6050_OK;
}//TODO Add ARG_ERR check

/**
  * @brief  Set MPU6050 gyroscope full scale range
  * @param  dev MPU6050 Handler
  * @param  gyro_range Use MPU6050_GYRO_FS_* defined values to select gyro full scale.
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetFullScaleGyroRange(MPU6050_HandleTypeDef *dev, uint8_t gyro_range)
{
	uint8_t data;
	if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_GYRO_CONFIG, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
	data &= 0b11100111;
	data |= ((gyro_range & 0b00000011) << MPU6050_GCONFIG_FS_SEL_BIT);
	if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, MPU6050_RA_GYRO_CONFIG, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;

	switch(gyro_range)
	{
		case MPU6050_GYRO_FS_250:
			dev->gyro_scale = 0.0076294;
			break;
		case MPU6050_GYRO_FS_500:
			dev->gyro_scale = 0.0152588;
			break;
		case MPU6050_GYRO_FS_1000:
			dev->gyro_scale = 0.0305176;
			break;
		case MPU6050_GYRO_FS_2000:
			dev->gyro_scale = 0.0610352;
			break;
		default:
            return MPU6050_ARG_ERR;
			break;
	}
  return MPU6050_OK;
}

/**
  * @brief  Set MPU6050 accelerometer full scale range
  * @param  dev MPU6050 Handler
  * @param  accel_range Use MPU6050_ACCEL_FS_* defined values to select gyro full scale.
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetFullScaleAccelRange(MPU6050_HandleTypeDef *dev, uint8_t accel_range)
{
	uint8_t data;
	if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_ACCEL_CONFIG, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
	data &= 0b11100111;
	data |= ((accel_range & 0b00000011) << MPU6050_ACONFIG_AFS_SEL_BIT);
	if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, MPU6050_RA_ACCEL_CONFIG, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;

	switch(accel_range)
	{
		case MPU6050_ACCEL_FS_2:
			dev->accel_scale = 0.0610352;
			break;
		case MPU6050_ACCEL_FS_4:
			dev->accel_scale = 0.1220703;
			break;
		case MPU6050_ACCEL_FS_8:
			dev->accel_scale = 0.2441406;
			break;
		case MPU6050_ACCEL_FS_16:
			dev->accel_scale = 0.4882813;
			break;
		default:
            return MPU6050_ARG_ERR;
			break;
	}
  return MPU6050_OK;
}

/**
  * @brief  Get MPU6050 gyroscope X-axis raw result directly
  * @param  dev MPU6050 Handler
  * @param  gyro_x Pointer to int16_t variable in which result will be saved
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_GetRotationXRAW(MPU6050_HandleTypeDef *dev, int16_t *gyro_x)
{
	uint8_t data[2];
  if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_GYRO_XOUT_H, 1, data, 2, dev->i2c_timeout)) return MPU6050_I2C_ERR;
	*gyro_x = (((int16_t)data[0]) << 8) | data[1];
  return MPU6050_OK;
}

/**
  * @brief  Get MPU6050 gyroscope Y-axis raw result directly
  * @param  dev MPU6050 Handler
  * @param  gyro_x Pointer to int16_t variable in which result will be saved
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_GetRotationYRAW(MPU6050_HandleTypeDef *dev, int16_t *gyro_y)
{
	uint8_t data[2];
  if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_GYRO_YOUT_H, 1, data, 2, dev->i2c_timeout)) return MPU6050_I2C_ERR;
	*gyro_y = (((int16_t)data[0]) << 8) | data[1];
  return MPU6050_OK;
}

/**
  * @brief  Get MPU6050 gyroscope Z-axis raw result directly
  * @param  dev MPU6050 Handler
  * @param  gyro_x Pointer to int16_t variable in which result will be saved
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_GetRotationZRAW(MPU6050_HandleTypeDef *dev, int16_t *gyro_z)
{
	uint8_t data[2];
  if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_GYRO_ZOUT_H, 1, data, 2, dev->i2c_timeout)) return MPU6050_I2C_ERR;
	*gyro_z = (((int16_t)data[0]) << 8) | data[1];
  return MPU6050_OK;
}

/**
  * @brief  Get MPU6050 gyroscope scaling factor
  * @param  dev MPU6050 Handler
  * @param  gyro_scale Pointer to float variable in which gyroscope scaling factor will be saved
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_GetGyroScale(MPU6050_HandleTypeDef *dev, float *gyro_scale)
{
	*gyro_scale = dev->gyro_scale;
  return MPU6050_OK;
}

/**
  * @brief  Get MPU6050 accelerometer X-axis raw result directly
  * @param  dev MPU6050 Handler
  * @param  gyro_x Pointer to int16_t variable in which result will be saved
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_GetAccelerationXRAW(MPU6050_HandleTypeDef *dev, int16_t *accel_x)
{
	uint8_t data[2];
	if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_ACCEL_XOUT_H, 1, data, 2, dev->i2c_timeout)) return MPU6050_I2C_ERR;
	*accel_x = (((int16_t)data[0]) << 8) | data[1];
  return MPU6050_OK;
}

/**
  * @brief  Get MPU6050 accelerometer Y-axis raw result directly
  * @param  dev MPU6050 Handler
  * @param  gyro_x Pointer to int16_t variable in which result will be saved
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_GetAccelerationYRAW(MPU6050_HandleTypeDef *dev, int16_t *accel_y)
{
	uint8_t data[2];
	if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_ACCEL_YOUT_H, 1, data, 2, dev->i2c_timeout)) return MPU6050_I2C_ERR;
	*accel_y = (((int16_t)data[0]) << 8) | data[1];
  return MPU6050_OK;
}

/**
  * @brief  Get MPU6050 accelerometer Z-axis raw result directly
  * @param  dev MPU6050 Handler
  * @param  gyro_x Pointer to int16_t variable in which result will be saved
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_GetAccelerationZRAW(MPU6050_HandleTypeDef *dev, int16_t *accel_z)
{
	uint8_t data[2];
	if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_ACCEL_ZOUT_H, 1, data, 2, dev->i2c_timeout)) return MPU6050_I2C_ERR;
	*accel_z = (((int16_t)data[0]) << 8) | data[1];
  return MPU6050_OK;
}

/**
  * @brief  Get MPU6050 accelerometer scaling factor
  * @param  dev MPU6050 Handler
  * @param  gyro_scale Pointer to float variable in which accelerometer scaling factor will be saved
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_GetAccelScale(MPU6050_HandleTypeDef *dev, float *accel_scale)
{
	*accel_scale = dev->accel_scale;
  return MPU6050_OK;
}

/**
  * @brief  Set MPU6050 internal sample rate divider. This division only works when base Fs = 1kHz. Refer to page 12,14 and 15 of register map for details
  * @param  dev MPU6050 Handler
  * @param  div Sample rate division
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetSampleRateDiv(MPU6050_HandleTypeDef *dev, uint8_t div){
  if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, MPU6050_RA_SMPLRT_DIV, 1, &div, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  return MPU6050_OK;
}

/**
  * @brief  Set MPU6050 INT pin active level
  * @param  dev  MPU6050 Handler
  * @param  level Use MPU6050_INTLVL_* to configure INT pin active level
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetIntPinActiveLevel(MPU6050_HandleTypeDef *dev, uint8_t level){
  uint8_t data;
  if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_INT_PIN_CFG, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  data &= ~(1<<MPU6050_INTCFG_INT_LEVEL_BIT);
  data |= ((level & 0b00000001) << MPU6050_INTCFG_INT_LEVEL_BIT);
  if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, MPU6050_RA_INT_PIN_CFG, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  return MPU6050_OK;
}

/**
  * @brief  Set MPU6050 INT pin mode
  * @param  dev  MPU6050 Handler
  * @param  mode Use MPU6050_INTDRV_* to configure INT pin mode
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetIntPinMode(MPU6050_HandleTypeDef *dev, uint8_t mode){
  uint8_t data;
  if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_INT_PIN_CFG, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  data &= ~(1<<MPU6050_INTCFG_INT_PINMODE_BIT);
  data |= ((mode & 0b00000001) << MPU6050_INTCFG_INT_PINMODE_BIT);
  if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, MPU6050_RA_INT_PIN_CFG, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  return MPU6050_OK;
}

/**
  * @brief  Set MPU6050 INT pin latching
  * @param  dev  MPU6050 Handler
  * @param  mode Use MPU6050_INTLATCH_* to configure INT pin latching
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetIntPinLatch(MPU6050_HandleTypeDef *dev, uint8_t mode){
  uint8_t data;
  if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_INT_PIN_CFG, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  data &= ~(1<<MPU6050_INTCFG_LATCH_INT_EN_BIT);
  data |= ((mode & 0b00000001) << MPU6050_INTCFG_LATCH_INT_EN_BIT);
  if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, MPU6050_RA_INT_PIN_CFG, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  return MPU6050_OK;
}

/**
  * @brief  Set MPU6050 INT status clear mode
  * @param  dev  MPU6050 Handler
  * @param  mode Use MPU6050_INTCLEAR_* to configure INT pin clearing
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetIntPinClearMode(MPU6050_HandleTypeDef *dev, uint8_t mode){
  uint8_t data;
  if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_INT_PIN_CFG, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  data &= ~(1<<MPU6050_INTCFG_INT_RD_CLEAR_BIT);
  data |= ((mode & 0b00000001) << MPU6050_INTCFG_INT_RD_CLEAR_BIT);
  if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, MPU6050_RA_INT_PIN_CFG, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  return MPU6050_OK;
}

/**
  * @brief  Enable/Disable MPU6050 raw data ready interrupt
  * @param  dev  MPU6050 Handler
  * @param  mode Use MPU6050_INTRAWREADY_* to enable or disable RAW_RDY interrupt
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_EnableRawReadyInt(MPU6050_HandleTypeDef *dev, uint8_t mode){
  uint8_t data;
  if(HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_INT_ENABLE, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  data &= ~(1<<MPU6050_INTEN_RAWREADY_BIT);
  data |= ((mode & 0b00000001) << MPU6050_INTEN_RAWREADY_BIT);
  if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, MPU6050_RA_INT_ENABLE, 1, &data, 1, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  return MPU6050_OK;
}

/**
  * @brief  Measure average from data
  * @param  data Pointer to array of values to calculate average from
  * @param  samples Number of values in data
  * @retval Calculated average
  */
static int16_t MPU6050_CalculateAverage(const int16_t *data, uint32_t samples){
  int32_t result = 0;
  for(uint32_t i=0; i<samples; i++){
    result += (int32_t)data[i];
  }
  result = result - (result % (int32_t)samples);
  result /= (int32_t)samples;
  //result *= 4; //TODO Very very bad
  return (int16_t)result;
}

/**
  * @brief  Measure MPU6050 gryo offset in X axis
  * @param  dev  MPU6050 Handler
  * @param  offset Pointer to int16_t variable in which measured offset will be saved
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_MeasureGyroOffsetX(MPU6050_HandleTypeDef *dev, int16_t *offset){
  MPU6050_StatusTypeDef ret = MPU6050_OK;
  int16_t data[MPU6050_GYRO_OFFSET_SAMPLES] = {0};
  for(uint32_t i=0; i<MPU6050_GYRO_OFFSET_SAMPLES; i++){
    ret = MPU6050_GetRotationXRAW(dev, &data[i]);
    if(ret != MPU6050_OK) return ret;
  }
  *offset = (-1)*MPU6050_CalculateAverage(data, MPU6050_GYRO_OFFSET_SAMPLES);
  return MPU6050_OK;
}

/**
  * @brief  Measure MPU6050 gryo offset in Y axis
  * @param  dev  MPU6050 Handler
  * @param  offset Pointer to int16_t variable in which measured offset will be saved
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_MeasureGyroOffsetY(MPU6050_HandleTypeDef *dev, int16_t *offset){
  MPU6050_StatusTypeDef ret = MPU6050_OK;
  int16_t data[MPU6050_GYRO_OFFSET_SAMPLES] = {0};
  for(uint32_t i=0; i<MPU6050_GYRO_OFFSET_SAMPLES; i++){
    ret = MPU6050_GetRotationYRAW(dev, &data[i]);
    if(ret != MPU6050_OK) return ret;
  }
  *offset = MPU6050_CalculateAverage(data, MPU6050_GYRO_OFFSET_SAMPLES);
  return MPU6050_OK;
}

/**
  * @brief  Measure MPU6050 gryo offset in Z axis
  * @param  dev  MPU6050 Handler
  * @param  offset Pointer to int16_t variable in which measured offset will be saved
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_MeasureGyroOffsetZ(MPU6050_HandleTypeDef *dev, int16_t *offset){
  MPU6050_StatusTypeDef ret = MPU6050_OK;
  int16_t data[MPU6050_GYRO_OFFSET_SAMPLES] = {0};
  for(uint32_t i=0; i<MPU6050_GYRO_OFFSET_SAMPLES; i++){
    ret = MPU6050_GetRotationZRAW(dev, &data[i]);
    if(ret != MPU6050_OK) return ret;
  }
  *offset = MPU6050_CalculateAverage(data, MPU6050_GYRO_OFFSET_SAMPLES);
  return MPU6050_OK;
}

/**
  * @brief  Set MPU6050 gryo offset in X axis
  * @param  dev  MPU6050 Handler
  * @param  offset Offset to set
  * @retval MPU6050 status
  */
MPU6050_StatusTypeDef MPU6050_SetGyroOffsetX(MPU6050_HandleTypeDef *dev, int16_t offset){
  uint8_t data[2];

  volatile int16_t test = (offset)>>8; //TODO test
  volatile uint16_t test2 = 0; //TODO test

  test2 = ((uint16_t)offset) >> 8; //TODO test

  data[0] = (uint8_t)(((uint16_t)offset)>>8);
  data[1] = (uint8_t)(offset);

  if(HAL_I2C_Mem_Write(dev->i2c_handle, dev->i2c_address, MPU6050_RA_GYRO_XOFFSET_H, 1, data, 2, dev->i2c_timeout)) return MPU6050_I2C_ERR;
  
  data[0] = 0;
  data[1] = 0;

  HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_GYRO_XOFFSET_H, 1, &data[0], 2, dev->i2c_timeout); //TODO test
  HAL_I2C_Mem_Read(dev->i2c_handle, dev->i2c_address, MPU6050_RA_GYRO_XOFFSET_L, 1, &data[1], 2, dev->i2c_timeout); //TODO test
  HAL_Delay(1); //TODO Test
  return MPU6050_OK;
}

