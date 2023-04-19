#ifndef __GIMBAL_MOTOR_H
#define __GIMBAL_MOTOR_H

#include "bsp_can.h"
#include "stm32f407xx.h"
#include "rmmotor.h"
#include "can.h"

//抬升电机
#define Lift_Motor_ID 0x201




//前伸电机速度环和位置速度环PID
#define Forword_Motor1_Spid_P 1.0f
#define Forword_Motor1_Spid_I 0.0f
#define Forword_Motor1_Spid_D 0.0f
#define Forword_Motor1_Ppid_P 1.0f
#define Forword_Motor1_Ppid_I 0.0f
#define Forword_Motor1_Ppid_D 0.0f

#define Forword_Motor2_Spid_P 1.0f
#define Forword_Motor2_Spid_I 0.0f
#define Forword_Motor2_Spid_D 0.0f
#define Forword_Motor2_Ppid_P 1.0f
#define Forword_Motor2_Ppid_I 0.0f
#define Forword_Motor2_Ppid_D 0.0f
//机械臂电机速度环和位置速度环PID

#define Pitch_3508_Spid_P 6.0f
#define Pitch_3508_Spid_I 0.0f
#define Pitch_3508_Spid_D 1.0f

#define Pitch_3508_Ppid_P 1.0f
#define Pitch_3508_Ppid_I 0.0f
#define Pitch_3508_Ppid_D 0.5f

#define Roll_Spid_P 3.0f
#define Roll_Spid_I 0.00f
#define Roll_Spid_D 0.0f

#define Roll_Ppid_P 1.0f
#define Roll_Ppid_I 0.0f
#define Roll_Ppid_D 1.0f


#define Yaw_6020_Spid_P 20.0f
#define Yaw_6020_Spid_I 0.0f
#define Yaw_6020_Spid_D 0.0f


#define Yaw_6020_Ppid_P 50.0f
#define Yaw_6020_Ppid_I 0.0f
#define Yaw_6020_Ppid_D 2000.0f

#define Tranverse_2006_Spid_P 1.0f
#define Tranverse_2006_Spid_I 0.01f
#define Tranverse_2006_Spid_D 0.0f

#define Tranverse_2006_Ppid_P 1.0f
#define Tranverse_2006_Ppid_I 0.01f
#define Tranverse_2006_Ppid_D 0.0f

#define LastJoint_2006_Spid_P 3.0f
#define LastJoint_2006_Spid_I 0.0f
#define LastJoint_2006_Spid_D 1.0f

#define LastJoint_2006_Ppid_P 1.0f
#define LastJoint_2006_Ppid_I 0.0f
#define LastJoint_2006_Ppid_D 1.0f


typedef __packed struct
{
    Motor_t Lift_Motor[2];
//    VL53L0_t *VL53L0;
    Encoder_t*(*Get_Encoder)(uint8_t);
//    VL53L0_t*(*Get_VL53L0_t)();
//    Encoder_t *Rotary_Encoder;
}Lift_UP_t;


typedef __packed struct
{
    Motor_t Pitch_Motor;//pitch轴/x轴3508电机
    Motor_t Roll_Motor; //roll轴/z轴2006/3508电机
    Motor_t Yaw_Motor;  //yaw轴/y轴6020电机
    Motor_t Traverse_Motor;//横移的2006电机
    Motor_t Last_Joint[2];    //最后一个自由度的两个2006
    Motor_t Forward_Motor[2];    //前伸电机x2 3508电机
    bool_t Suker_state; //吸盘状态
    bool_t is_Auto_Mode;   //机械臂模式 自动/手动
    Encoder_t *(*Get_Encoder)(uint8_t);
} Three_D_Arm_t;

typedef __packed struct{
    Motor_t Clip_Motor[2];
    Encoder_t *(*Get_Encoder)(uint8_t);
}Clip_Module_t;


typedef __packed struct
{
    Lift_UP_t *Lift_t;  //抬升结构体
    Three_D_Arm_t *TD_t;//机械臂结构体
    Clip_Module_t *C_t;
} Grasp_t; //云台上全部夹爪结构体

Grasp_t* Return_Grasp_t_Pointer(void);
void Gimbal_Motor_Init(void);
void Forword_Motor_Drive(Three_D_Arm_t *TD,int32_t X_IN);
void Gimbal_PowerOff_Drive(Grasp_t *G);
void Arms_Drive(Three_D_Arm_t *Arm_t,  int16_t roll, int16_t pitch, int16_t yaw,int16_t joint ,int16_t  forward, bool_t update_sucker_state) ;
void Three_Degrees_Arms_Init();
void MotorVelocityCurve(Motor_t *curve);
static void CalCurveSPTA(Motor_t *spta);
#endif // !__GIMBAL_MOTOR_H

