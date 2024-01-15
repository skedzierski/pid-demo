#include "cmsis_os.h"

typedef enum {
    MPU6050,
    VL6180
} source;

typedef struct {
    source device;
    union {
        struct {
            float gyro_x; 
            float acc_x;
        } vec2;
        int32_t distance;
    };
    uint32_t time_stamp;
} measurment;

void demo_tof(void* args);
void demo_acc(void* args);
void simple_logger(void* args);