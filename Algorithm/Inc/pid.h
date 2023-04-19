#ifndef __PID_H
#define __PID_H
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "main.h"
#include "struct_typedef.h"
/*---------CHASSIS---------*/
// #define Chassis_DeadArea    0
// #define Chassis_ISeparate   5000

// #define Motor1_Kp 					5.0f
// #define Motor1_Ki 					0.00f
// #define Motor1_Kd 					0.02f
// #define Motor1_Ki_Limit 		10.0f
// #define Motor1_Output_Limit 12000.0f

// #define Motor2_Kp 					5.0f
// #define Motor2_Ki 					0.00f
// #define Motor2_Kd 					0.02f
// #define Motor2_Ki_Limit 		10.0f
// #define Motor2_Output_Limit 12000.0f

// #define Motor3_Kp 					5.0f
// #define Motor3_Ki 					0.00f
// #define Motor3_Kd 					0.02f
// #define Motor3_Ki_Limit 		10.0f
// #define Motor3_Output_Limit 12000.0f

// #define Motor4_Kp 					5.0f
// #define Motor4_Ki 					0.00f
// #define Motor4_Kd 					0.02f
// #define Motor4_Ki_Limit 		10.0f
// #define Motor4_Output_Limit 12000.0f

// #define Gimbal_Kp 					10.0f       //��̨
// #define Gimbal_Ki 					0.001f
// #define Gimbal_Kd 					0.5f
// #define Gimbal_Ki_Limit 		100.0f
// #define Gimbal_Output_Limit 5000.0f

// #define Rescue_Card_Kp      				3.0f
// #define Rescue_Card_Ki							0.0f
// #define Rescue_Card_Kd      				0.1f
// #define Rescue_Card_Ki_Limit				10.0f
// #define Rescue_Card_Output_Limit 		3000.0f

// #define Rescue_Claw_Kp      				4.0f
// #define Rescue_Claw_Ki							0.0f
// #define Rescue_Claw_Kd      				0.02f
// #define Rescue_Claw_Ki_Limit				10.0f
// #define Rescue_Claw_Output_Limit 		3000.0f

// #define Yaw_Kp 							10.0f
// #define Yaw_Ki 							0.0f
// #define Yaw_Kd 							0.02f
// #define Yaw_Ki_Limit 				10.0f
// #define Yaw_Output_Limit 		12000.0f


typedef __packed struct
{
    fp32 kp;//比例项
    fp32 ki;//积分项
    fp32 kd;//微分项

    fp32 P_Out;
    fp32 I_out;
    fp32 D_Out;
    fp32 max_iout;//最大积分输出

    fp32 error[2];//第一个是上一次的差值，第二个是最新的差值

    fp32 SetValues;//设定值
    fp32 Last_CurrentValues;
    fp32 CurrentValues;//当前值 

    fp32 max_output;//上限

    fp32 stepIn;    //步进式的

    /* 变积分 */
    fp32 errorabsmax; //偏差绝对值最大值
    fp32 errorabsmin; //偏差绝对值最小值

    fp32 Ierror;
    fp32 Derror[3];  //微分项 0最新 1上一次 2上上次
	//删掉冗余的error
    fp32 Output;
}PIDTypeDef;

fp32 PID_Position_speed_Deal(PIDTypeDef *SpeedPid, PIDTypeDef *PoisionPid);
fp32 PIDlimit(fp32 object,fp32 H_Limitation,fp32 L_Limitation);
fp32 PIDcal(PIDTypeDef *pid);
// fp32 PID_Position_speed_Control(PIDTypeDef *SpeedPid,PIDTypeDef *PoisionPid,fp32 current_speed,fp32 current_poision,fp32 set_poision);
fp32 PID_STOP_Init(void);
void PID_Init(PIDTypeDef *PID_Init_t, fp32 kp, fp32 ki, fp32 kd, float Output_Limit, fp32 max_iout);
float step_in_processing(PIDTypeDef *vPID, float sp);
int32_t PID_regulator(PIDTypeDef *vPID, float actualValue);

#endif



