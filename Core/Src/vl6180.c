#include "vl6180.h"
#include "main.h"
#include "stdio.h"

static void wait_ready(VL1680_t *hvl1680);
static VL1680_Status wait_boot(VL1680_t *hvl1680);

VL1680_Status VL1680_Init(VL1680_t* hvl1680)
{
    hvl1680->dev_addr=VL1680_I2C_ADDRESS;
    hvl1680->hi2c = &I2C_HANDLE;
    VL1680_Status status = VL1680_OK;
    HAL_GPIO_WritePin(VL6180_GPIO0_GPIO_Port, VL6180_GPIO0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(VL6180_GPIO0_GPIO_Port, VL6180_GPIO0_Pin, GPIO_PIN_SET);
    status += wait_boot(hvl1680);
    status += VL1680_SetAveragingPeriod(hvl1680, 48);
    status += VL1680_DisableEarlyConverganceEstimate(hvl1680);
    status += VL1680_SetRangingMode(hvl1680, SYSRANGE__MODE_SINGLE);

    return status;
}

VL1680_Status VL1680_Init_IT(VL1680_t *hvl1680)
{
    VL1680_Status status = VL1680_OK;
    status += VL1680_Init(hvl1680);
    status += VL1680_SetInterMeasurmentPeriod(hvl1680, 10);
    status += VL1680_SetInterruptMode(hvl1680, RANGE_INT_NEW_SAMPLE_RDY);
    status += VL1680_SetGPIO1Mode(hvl1680, GPIO_INT_OUTPUT | GPIO_ACTIVE_HIGH);
    status += VL1680_SetRangingMode(hvl1680, SYSRANGE__MODE_CONTINOUS);
    status += VL1680_StartRange(hvl1680);
    return status;
}

uint8_t VL1680_PollMeasurment(VL1680_t *hvl1680)
{
    VL1680_StartRange(hvl1680);
    wait_ready(hvl1680);
    uint8_t distance = 0;
    read_byte(hvl1680, RESULT__RANGE_RAW, &distance);
    return distance;
}

static VL1680_Status wait_boot(VL1680_t *hvl1680)
{
    // c HAL_Delay(1000);
    VL1680_Status status = VL1680_OK;
    uint8_t is_out_of_reset = 0;
    while(!(is_out_of_reset & 1))
    {
        status = read_byte(hvl1680, SYSTEM__FRESH_OUT_OF_RESET, &is_out_of_reset);
        if(status != VL1680_OK)
            return status;
    }
    return status;
}

static void wait_ready(VL1680_t *hvl1680)
{
    uint8_t reg_val;
    uint8_t is_ready = 0;
    while(!is_ready)
    {
        read_byte(hvl1680, RESULT__RANGE_STATUS, &reg_val);
        is_ready = reg_val & 1;
        printf("not ready\n");
    }
}

VL1680_Status read_byte(VL1680_t* hvl1680, uint16_t reg_addr, uint8_t* data)
{
    HAL_StatusTypeDef stat;
    uint8_t payload[2] = {(reg_addr & 0xFF00) >> 8, reg_addr & 0x00FF};
    stat = HAL_I2C_Master_Transmit(hvl1680->hi2c, hvl1680->dev_addr, payload, 2, 1000);
    
    if(stat != HAL_OK)
        return VL1680_I2C_WRITE_ERR;

    stat = HAL_I2C_Master_Receive(hvl1680->hi2c, hvl1680->dev_addr, data, 1, 1000);
    if(stat != HAL_OK)
        return VL1680_I2C_READ_ERR;
    
    return VL1680_OK;
}

VL1680_Status read16(VL1680_t* hvl1680, uint16_t reg_addr, uint16_t* data)
{
    return 1;
}

VL1680_Status read32(VL1680_t* hvl1680, uint16_t reg_addr, uint32_t* data)
{
    return 1;
}

VL1680_Status write16(VL1680_t* hvl1680, uint16_t reg_addr, uint16_t data)
{
    return 1;
}

VL1680_Status write_byte(VL1680_t* hvl1680, uint16_t reg_addr, uint8_t data)
{
    uint8_t payload[3] = {(reg_addr & 0xFF00) >> 8, reg_addr & 0x00FF, data};
    HAL_StatusTypeDef stat;
    stat = HAL_I2C_Master_Transmit(hvl1680->hi2c, hvl1680->dev_addr, payload, 2, 1000);
    
    if(stat != HAL_OK)
        return VL1680_I2C_WRITE_ERR;
    return VL1680_OK;
}

VL1680_Status VL1680_SetAveragingPeriod(VL1680_t* hvl1680, uint8_t period)
{
    if(period >= 0 && period <= 255)
        return write_byte(hvl1680, READOUT__AVERAGING_SAMPLE_PERIOD, period);
    else
        return VL1680_INVARG;
}

VL1680_Status VL1680_SetRangingMode(VL1680_t* hvl1680, uint8_t ranging_mode)
{
    if(ranging_mode == SYSRANGE__MODE_SINGLE || ranging_mode == SYSRANGE__MODE_SINGLE)
        return  write_byte(hvl1680, SYSRANGE__START, ranging_mode);
    else
        return VL1680_INVARG;
}

VL1680_Status VL1680_SetInterMeasurmentPeriod(VL1680_t* hvl1680, uint8_t ms)
{
    if(ms % 10 == 0 && ms >= 0 && ms <= 250)
        return write_byte(hvl1680, SYSRANGE__INTERMEASUREMENT_PERIOD, ms/10);
    else 
        return VL1680_INVARG;
}

VL1680_Status VL1680_SetInterruptMode(VL1680_t* hvl1680, uint8_t mode)
{
    if(mode >= 0 && mode <= 4)
        return write_byte(hvl1680, SYSTEM__INTERRUPT_CONFIG_GPIO, mode);
    else
        return VL1680_INVARG;
}

VL1680_Status VL1680_SetGPIO1Mode(VL1680_t* hvl1680, uint8_t mode)
{
    VL1680_Status status = VL1680_OK;
    if(mode <= 31)
    {
        status += write_byte(hvl1680, DATA_BEING_UPDATED, SYSTEM__GROUPED_PARAMETER_HOLD);
        status += write_byte(hvl1680, SYSTEM__MODE_GPIO1, GPIO_INT_OUTPUT | GPIO_ACTIVE_HIGH);
        status += write_byte(hvl1680, DATA_IS_STABLE, SYSTEM__GROUPED_PARAMETER_HOLD);
        return status;
    }
    else 
        return VL1680_INVARG;
}

VL1680_Status VL1680_StartRange(VL1680_t* hvl1680)
{
    return write_byte(hvl1680, SYSRANGE__START, SYSRANGE__START_RANGE);
}

VL1680_Status VL1680_DisableEarlyConverganceEstimate(VL1680_t* hvl1680)
{
    uint8_t enables;
    VL1680_Status status = 0;
    status += read_byte(hvl1680, SYSRANGE__RANGE_CHECK_ENABLES, &enables);
    enables &= ~(enables & 1);
    status += write_byte(hvl1680, SYSRANGE__RANGE_CHECK_ENABLES, enables);
    return status;
}