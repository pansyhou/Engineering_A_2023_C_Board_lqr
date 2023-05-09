#ifndef __RMMOTOR_H
#define __RMMOTOR_H


#include "pid.h"

#include "bsp_Motor_Encoder.h"
#include "struct_typedef.h"


/*电机种类枚举*/
typedef enum
{
		CHASSIS_M,		 //底盘
		PITCH_M,			 //Pitch
		YAW_M,  			 //Yaw
		SWING_M,			 //翻转
		CLAMP_M,			 //夹取
		TRANS_M,			 //传送
		SMOOTH_M,			 //滑轨
		SLIDE_M,		 	 //抬升
		RESCUE_M,      //救援
		SUPPLY_M,			 //补给
		AMMUNITI_M,		 //拨弹
		CURRENCY_M     //通用
}MotorType_e;

typedef enum SpeedCurve {
    CURVE_NONE=0,  //直启
    CURVE_TRAP=1,  //梯形曲线
    CURVE_SPTA=2  //S型曲线
}SpeedCurveType;

/*电机结构体*/
typedef __packed struct
{
	uint8_t Pos_Lock;			//位置锁
	int32_t ExpSpeed;			//期望速度
	float ExpRadian;			//期望角度
	pid_parameter_t SPID;			//速度环PID
	pid_parameter_t PPID;			//位置环PID
	Encoder_t *Encoder;		//码盘
	MotorType_e MotorType;//电机种类
	int16_t temp;	//温度
	int16_t anper;	//当前电流值
	float torque;   //力矩
	//还在想，电机还能封装什么方法进去
	//TODO:
//
//    float startSpeed;//开始调速时的初始速度
//    float targetSpeed;   //目标速度
//    float stepSpeed;    //加速度
//    float speedMax;     //最大速度
//    float speedMin;     //最小速度
//    float currentSpeed;//当前速度
//    uint32_t aTimes;    //调速时间
//    uint32_t maxTimes;   //调速跨度
//    SpeedCurveType curveMode;  //曲线类型
//    float flexible;     //S曲线拉伸度
}Motor_t;

/**************函数声明**************/
float Motor_Torque_Deal(Motor_t *motor);
void MotorValZero(Motor_t *motor);
//int16_t RM_Speed_PID(PIDTypeDef *SpeedPID_t, int16_t Set_speed, int16_t Actual_Speed, int16_t limit_values);//简简单单速度环PID控制
//int16_t RM_Speed_Position_PID(PIDTypeDef *SpeedPID_t, PIDTypeDef *PositionPID_t, int16_t Actual_Speed, int16_t Actual_Position, int16_t SetPosition, int16_t limit_values);//位置速度闭环控制
//void    RM_Motor_Actual_Poisition(CAN_Motor_Measure_Data *rmMotor, int16_t gear_Ratio, int16_t lap_encoder);//计算真实的码盘值
//int16_t RM_Angle_Limiting_Int16(int16_t Angle_Err, int16_t lap_encoder);//临角处理16位（对应角度正值）
//int32_t RM_Check_CodeValue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder);//过临界值复位码盘值 （限制于360度的码盘值循环 DJI电机）
//int16_t RM_Check_Motor_Block(int16_t position);//堵转检测
//float   RM_Get_Yaw_AngleDifference(const CAN_Motor_Measure_Data *Yaw_Motor, int16_t Ratio);//Yaw轴电机与底盘中心差角获取
//int16_t RM_Encoder_Real(int32_t read_code);//多圈绝对值编码器数据转换
//int16_t RM_Yaw_Actual_Code_Conversion(int16_t actual_code, int16_t max_code, int16_t middle_code);//Yaw轴真实位置（多圈编码器）（左正右负）
//int16_t Motor_Position_Stepping(PIDTypeDef *S_pid, PIDTypeDef *P_pid, int16_t actual_Position, int16_t actual_speed, int16_t set_Position, int16_t current_limit);


#endif

