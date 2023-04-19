/************************** Dongguan-University of Technology -ACE**************************
* @file rmmotor.c
* @author pansyhou侯文辉 (1677195845lyb@gmail.com)
* @brief
*      命名格式：
*              RM：rmmotor
* @version 0.1
* @date 2022-02-11
*
* @copyright Copyright (c) 2022
*
************************** Dongguan-University of Technology -ACE***************************/

#include "rmmotor.h"
#include "string.h"
//#include "SYSInit.h"

/*电机结构体清零*/
void MotorValZero(Motor_t *motor)
{
    memset((void *) motor, 0, sizeof(Motor_t));
}


float Motor_Torque_Deal(Motor_t *motor)
{
    float torque = (9550 * (20 * (motor->anper / 20))) / motor->Encoder->Speed[1];
    motor->torque = torque;
    return torque;
}
///************************** Dongguan-University of Technology -ACE**************************
// * @brief 简简单单速度环PID控制
// *
// * @param SpeedPID_t 速度控制环PID结构体
// * @param Set_speed 设置速度
// * @param Actual_Speed 实际速度
// * @param limit_values 限制电流值
// * @return int16_t 输出电流值
// ************************** Dongguan-University of Technology -ACE***************************/
//int16_t RM_Speed_PID(PIDTypeDef *SpeedPID_t, int16_t Set_speed, int16_t Actual_Speed, int16_t limit_values)
//{
//    int16_t output;

//    //更新结构体里的数据
//    SpeedPID_t->SetValues = Set_speed;
//    SpeedPID_t->CurrentValues = Actual_Speed;

//    //限制输出
//    SpeedPID_t->H_Limited = limit_values;
//    SpeedPID_t->L_Limited = -limit_values;

//    output = PIDcal(SpeedPID_t);

//    return output;
//}

///************************** Dongguan-University of Technology -ACE**************************
// * @brief 位置速度闭环控制
// *          多半会有bug，需要测试。
// * @param SpeedPID_t
// * @param PositionPID_t
// * @param Actual_Speed
// * @param Actual_Position
// * @param SetPosition
// * @param limit_values
// * @return int16_t
// ************************** Dongguan-University of Technology -ACE***************************/
//int16_t RM_Speed_Position_PID(PIDTypeDef *SpeedPID_t, PIDTypeDef *PositionPID_t, int16_t Actual_Speed, int16_t Actual_Position, int16_t SetPosition, int16_t limit_values)
//{
//    int16_t output;

//    //更新结构体里的数据
//    SpeedPID_t->CurrentValues = Actual_Speed;

//    //限制输出
//    SpeedPID_t->H_Limited = limit_values;
//    SpeedPID_t->L_Limited = -limit_values;
//    //这段的位置限制还要再细调
//    PositionPID_t->H_Limited = limit_values;
//    PositionPID_t->L_Limited = -limit_values;

//    output = PID_Position_speed_Control(SpeedPID_t, PositionPID_t, Actual_Speed, Actual_Position, SetPosition);

//    return output;
//}

///************************** Dongguan-University of Technology -ACE**************************
// * @brief
// *
// * @param S_pid
// * @param P_pid
// * @param actual_Position
// * @param actual_speed
// * @param set_Position
// * @param current_limit
// * @return int16_t
// ************************** Dongguan-University of Technology -ACE***************************/
//int16_t Motor_Position_Stepping(PIDTypeDef *S_pid, PIDTypeDef *P_pid, int16_t actual_Position, int16_t actual_speed, int16_t set_Position, int16_t current_limit)
//{
//    int32_t output;

//    step_in_processing(P_pid,set_Position);//步进式PID， 对设置值进行平滑处理

//    S_pid->SetValues=PID_regulator(P_pid,actual_Position);//速度环设定值由位置环处理

//    S_pid->CurrentValues=actual_speed;
//    S_pid->H_Limited=current_limit;
//    S_pid->L_Limited=-current_limit;

//    output=PIDcal(S_pid);
//    
//    return output;
//}



///************************** Dongguan-University of Technology -ACE**************************
//* @brief RM电机一般算法
//*          作用：计算真实的码盘值(通用，无论是什么电机或编码器，只要涉及到单圈增量式的都可以用)
//*
//* @param rmMotor 电机数据结构体
//* @param gear_Ratio 减速比乘传动比
//* @param lap_encoder 单圈码盘值
//************************** Dongguan-University of Technology -ACE***************************/
//void RM_Motor_Actual_Poisition(CAN_Motor_Measure_Data *rmMotor, int16_t gear_Ratio, int16_t lap_encoder)
//{
//   if (rmMotor->first_Flag == 0) //第一次进入时记录码盘值
//   {
//       rmMotor->last_position = rmMotor->position;
//       rmMotor->first_Flag = 1;
//   }
//   rmMotor->actual_Position += RM_Angle_Limiting_Int16(rmMotor->position - rmMotor->last_position, lap_encoder); //差值累加
//   rmMotor->actual_Position = RM_Check_CodeValue(rmMotor->actual_Position, gear_Ratio, lap_encoder);
//   rmMotor->last_position = rmMotor->position;
//}

///************************** Dongguan-University of Technology -ACE**************************
// * @brief 临角处理16位（对应角度正值）
// *
// * @param Angle_Err
// * @param lap_encoder
// * @return int16_t
// ************************** Dongguan-University of Technology -ACE***************************/
//int16_t RM_Angle_Limiting_Int16(int16_t Angle_Err, int16_t lap_encoder)
//{
//    if (Angle_Err < -(lap_encoder / 2))
//    {
//        Angle_Err += (lap_encoder - 1);
//    }
//    if (Angle_Err > (lap_encoder / 2))
//    {
//        Angle_Err -= (lap_encoder - 1);
//    }
//    return Angle_Err;
//}

///************************** Dongguan-University of Technology -ACE**************************
// * @brief 过临界值复位码盘值 （限制于360度的码盘值循环 DJI电机）
// *
// * @param value
// * @param gear_Ratio
// * @param lap_encoder
// * @return int32_t
// ************************** Dongguan-University of Technology -ACE***************************/
//int32_t RM_Check_CodeValue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder)
//{
//    if (value > (gear_Ratio * lap_encoder) / 2)
//    {
//        value = value - (gear_Ratio * lap_encoder);
//    }
//    if (value < (-(gear_Ratio * lap_encoder) / 2))
//    {
//        value = (gear_Ratio * lap_encoder) - value;
//    }

//    return value;
//}

///************************** Dongguan-University of Technology -ACE**************************
// * @brief 堵转检测
// *
// * @param position
// * @return int16_t
// ************************** Dongguan-University of Technology -ACE***************************/
//static int16_t Block_Cnt = 0;
//int16_t RM_Check_Motor_Block(int16_t position)
//{
//    static int16_t last_position;
//    if (int16_t_abs(last_position - position) < 10) //这个可以自己调
//        Block_Cnt++;
//    else
//        Block_Cnt = 0;
//    last_position = position;
//    if (Block_Cnt > 100)
//        return 1;
//    else
//        return 0;
//}

///************************** Dongguan-University of Technology -ACE**************************
// * 机器人坐标体系：
// * yaw轴：侧航角ψ （往左往右）。右偏航为正，反之为负。
// * roll轴：翻滚角ϕ （视角旋转）。向右滚为正，反之为负。
// * pitch轴：俯仰角θ \thetaθ（往上往下）。抬头为正，反之为负。
// * 图解：https://blog.csdn.net/sandalphon4869/article/details/103911254
// ************************** Dongguan-University of Technology -ACE***************************/

//float Yaw_Difference_Angle = 0;                   //云台底盘差角
//int32_t Yaw_Middle_Code = (8192 * YAW_RATIO) / 2; // Y轴总码盘的半值
///************************** Dongguan-University of Technology -ACE**************************
// * @brief Yaw轴电机与底盘中心差角获取
// *  以底盘中心为0度，向右为正角，左为负，大小范围为0-180度
// *
// * @param Yaw_Motor
// * @param Ratio
// * @return float
// ************************** Dongguan-University of Technology -ACE***************************/
//float RM_Get_Yaw_AngleDifference(const CAN_Motor_Measure_Data *Yaw_Motor, int16_t Ratio)
//{
//    Yaw_Difference_Angle = (Yaw_Motor->actual_Position * 360) / (8192 * Ratio);
//    return Yaw_Difference_Angle;
//}

//int16_t rotate_50_cirecle = 0;
//int16_t last_code = 0;
///************************** Dongguan-University of Technology -ACE**************************
// * @brief 多圈绝对值编码器数据转换
// *
// * @param read_code 真实码盘值
// * @return int16_t
// ************************** Dongguan-University of Technology -ACE***************************/
//int16_t RM_Encoder_Real(int32_t read_code)
//{
//    int code_output = 0;

//    if (read_code <= 49152) //晚点问问这个的含义
//    {
//        code_output = read_code % 3072;
//        return code_output;
//    }
//    else if (read_code > 49152)
//    {
//        return -1;
//    }
//    else
//    {
//        return 0;
//    }
//}

///************************** Dongguan-University of Technology -ACE**************************
// * @brief Yaw轴真实位置（多圈编码器）（左正右负）
// *
// * @param actual_code 读取到的真实单圈码盘值
// * @param max_code 单圈码盘总数
// * @param middle_code 中间位置的码盘值
// * @return int16_t 以中间位置为标准的临角处理位置值（左半圈正，右半圈负）
// ************************** Dongguan-University of Technology -ACE***************************/
//int16_t RM_Yaw_Actual_Code_Conversion(int16_t actual_code, int16_t max_code, int16_t middle_code)
//{
//    if (0 <= actual_code && actual_code <= middle_code) //将码盘值以中间为标准
//    {
//        last_code = max_code - (middle_code - actual_code);
//        if (last_code >= (max_code / 2) && last_code <= max_code) //临角处理
//            last_code = last_code - max_code;
//        else if (last_code >= 0 && last_code < (max_code / 2))
//            last_code = last_code;
//    }
//    else if (middle_code < actual_code && actual_code <= 3072)
//    {
//        last_code = actual_code - middle_code;
//        if (last_code >= (max_code / 2) && last_code <= max_code) //临角处理
//            last_code = last_code - max_code;
//        else if (last_code >= 0 && last_code < (max_code / 2))
//            last_code = last_code;
//    }
//    else
//        last_code = -1;

//    return last_code;
//}

