#include "tof_task.h"

void TOF_Init(TOF t, Mediator* m, VL53L0X_DEV dev)
{

}
void SERVO_Init(SERVO servo, Mediator* m, Servo_HandleTypeDef* servo_handle)
{

}
void PID_Init(PID pid_obj, Mediator* m, PIDController_t pid_ctrl)
{
}

void wait_range_ready(void* args)
{
    Mediator* m;
    uint8_t data_ready;
    while(1)
    {
        xTaskNotifyWait();
        for(;;)
        {
            VL53L0X_GetMeasurementDataReady(m->tof->Dev, &data_ready);
            vTaskDelay(2);
            if(data_ready)
            {
                Mediator_notify(e);
                xTaskNotify();
                break;
            }
        }
    }
}