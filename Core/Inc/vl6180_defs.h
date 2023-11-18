#ifndef VL6180_DEFS_H_
#define VL6180_DEFS_H_

#define VL1680_I2C_ADDRESS 0x52
#define I2C_HANDLE hi2c1

#define system__gpio0_is_xshutdown_enabled (1 << 6)
#define system__gpio0_is_xshutdown_disabled ~system__gpio0_is_xshutdown_enabled
#define system__gpio0_polarity_active_high 
#define system__gpio0_polarity_active_low ~system__gpio0_polarity_active_high
#define system__gpio0_select

#define SYSRANGE__MODE_SINGLE (0 << 1)
#define SYSRANGE__MODE_CONTINOUS (1 << 1)
#define SYSRANGE__START_RANGE 1

#define RESULT__RANGE_DEVICE_READY 0

#define DATA_BEING_UPDATED 1
#define DATA_IS_STABLE 0
#define GPIO_ACTIVE_HIGH (1 << 5)
#define GPIO_INT_OUTPUT (0b1000 << 1)

//interrupt modes
#define RANGE_INT_DISABLED 0
#define RANGE_INT_LEVEL_HIGH 1
#define RANGE_INT_LEVEL_LOW 2
#define RANGE_INT_OUT_OF_WINDOW 3
#define RANGE_INT_NEW_SAMPLE_RDY 4

#endif