#include "cmsis_os.h"

typedef enum {
    MPU6050,
    VL6180
} source;

typedef struct {
    source device;
    union {
        float gyro_x; 
        float acc_x;
        int32_t distance;
    };
} measurment;

void demo_tof(void* args);
void demo_acc(void* args);
void simple_logger(void* args);