#ifndef __MATHS_H
#define __MATHS_H

#define RAD2DEG     (57.295779513082320876798154814105f)  // 弧度转角度

#include "struct_typedef.h"

//运动加速度限制斜坡结构体
typedef __packed struct
{
    float Input;      //当前取样值
    float Last_Input; //上次取样值
    float Output;     //输出值
    float acc_now;    //当前加速度
    float acc_limit;  //需要限制的加速度
}Acceleration_Control_type_t;


//滑动均值滤波参数（浮点）
typedef __packed struct
{
    float Input;     //当前取样值
    int count_num;   //取样次数
    float Output;    //滤波输出
    float Sum;       //累计总和
    float FIFO[250]; //队列
    int sum_flag;    //已经够250个标志
} sliding_mean_filter_type_t;

//一阶低通滤波参数
typedef __packed struct
{
	fp32 input;        //输入数据
	fp32 last_input;   //上次数据
	fp32 out;          //滤波输出的数据
	fp32 num;          //滤波参数
}first_order_filter_type_t;



/***********函数声明*************/

int16_t limit(int16_t x,int16_t max,int16_t min);/*数值大小限制函数*/
int16_t int16_t_abs(int16_t x); // 绝对值计算
int16_t RETURN_MAX(int16_t*Value,int16_t num);//找出最大值
int32_t int32_limit(int32_t x, int32_t max, int32_t min);//数值限制

int16_t int16_limit(int16_t x, int16_t max, int16_t min);
float float_limit(float x, float max, float min);
signed long limit_long(signed long x, signed long max, signed long min);
signed long long_abs(signed long x);// 绝对值计算
int16_t Dead_Zone(int16_t Value,int16_t Limit);//死区处理
float float_abs(float x); // 绝对值计算
float invSqrt(float x) ;//平方根倒数速算法
int16_t Motion_Acceleration_Control(Acceleration_Control_type_t *acceleration_control, int16_t Input, int16_t Limit);
float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input);
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 num);
void Sliding_Mean_Filter_Init(sliding_mean_filter_type_t *mean_filter);
float Sliding_Mean_Filter(sliding_mean_filter_type_t *mean_filter, float Input, int num);
int16_t Loop_Restriction_Int16(int16_t num, int16_t limit_num);
float Loop_Restriction_Float(float num, float limit_num);
float loop_fp32_constrain(float Input, float minValue, float maxValue);
void Data_Acceleration_Control(float angle);
#endif

