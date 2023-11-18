#ifndef VL1680_H_
#define VL1680_H_

#include "vl6180_register_map.h"
#include "vl6180_defs.h"
#include "main.h"

#include <stdint.h>

extern I2C_HandleTypeDef I2C_HANDLE;
typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint16_t dev_addr;
} VL1680_t;

typedef enum {
    VL1680_OK,
    VL1680_ERR,
    VL1680_I2C_READ_ERR,
    VL1680_I2C_WRITE_ERR,
    VL1680_INVARG
} VL1680_Status;

VL1680_Status VL1680_Init(VL1680_t* hvl1680);
VL1680_Status VL1680_Init_IT(VL1680_t*);
uint8_t VL1680_PollMeasurment(VL1680_t *hvl1680);
void VL1680_Get_Measurment_Async(uint32_t*);
void VL1680_ISR(void);
VL1680_Status VL1680_SetAveragingPeriod(VL1680_t* hvl1680, uint8_t);
VL1680_Status VL1680_SetRangingMode(VL1680_t* hvl1680, uint8_t ranging_mode);
VL1680_Status VL1680_SetInterMeasurmentPeriod(VL1680_t* hvl1680, uint8_t ms);
VL1680_Status VL1680_SetInterruptMode(VL1680_t* hvl1680, uint8_t mode);
VL1680_Status VL1680_SetGPIO1Mode(VL1680_t* hvl1680, uint8_t mode);
VL1680_Status VL1680_StartRange(VL1680_t* hvl1680);
VL1680_Status VL1680_DisableEarlyConverganceEstimate(VL1680_t*);
VL1680_Status write_byte(VL1680_t* hvl1680, uint16_t reg_addr, uint8_t data);
VL1680_Status write16(VL1680_t* hvl1680, uint16_t reg_addr, uint16_t data);
VL1680_Status read_byte(VL1680_t* hvl1680, uint16_t reg_addr, uint8_t* data);
VL1680_Status read16(VL1680_t* hvl1680, uint16_t reg_addr, uint16_t* data);
VL1680_Status read32(VL1680_t* hvl1680, uint16_t reg_addr, uint32_t* data);

#endif
