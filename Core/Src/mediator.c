#include "mediator.h"


void Mediator_BalanceBallAt(Mediator* m, uint16_t pos)
{
    float control = 0;
    float setting_for_servo = 0;
    VL53L0X_PerformSingleRangingMeasurement(m->tof, &m->data);
    control = PID_GetNewControl(m->pid, m->data.RangeMilliMeter, pos);
    setting_for_servo = Adapter_Map(m->adapter, control);
    SERVO_SetPosition(m->servo, setting_for_servo);
}

uint16_t Mediator_GetRange(Mediator* m)
{
    return m->data.RangeMilliMeter;
}

void MediatorBuilder_AddPID(MediatorBuilder* builder, PIDController_t* pid)
{
    builder->m.pid = pid;
}

void MediatorBuilder_AddServo(MediatorBuilder* builder, Servo_HandleTypeDef* servo)
{
    builder->m.servo = servo;
}

void MediatorBuilder_AddTOF(MediatorBuilder* builder, VL53L0X_DEV tof)
{
    builder->m.tof = tof;
}

void MediatorBuilder_AddAdapter(MediatorBuilder* builder, Adapter_t* adpt)
{
    builder->m.adapter = adpt;
}

Mediator MediatorBuilder_GetMediator(MediatorBuilder* builder)
{
    return builder->m;
}
