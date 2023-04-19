#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H
#include "RemoteDeal.h"
#include "bsp_Distance_Sensor.h"
#include "GimbalMotor.h"
#include "fsm.h"


/**GImbal main structure **/

typedef __packed struct
{
    REMOTE_t *RC;   //遥控器主结构体
    FSM_t *Gimbal_Fsm;
    Grasp_t *Graps;
//    void (*Lift_Up_fun)(Lift_UP_t*,int16_t);
    void (*PowerOff)(Grasp_t*);//云台断电
}Gimbal_t;

static void Gimbal_Init(void);

void Gimbal_Task(void *pvParameters);
#endif // !__GIMBAL_TASK_H

