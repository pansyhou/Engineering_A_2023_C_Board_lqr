#include "maths.h"
#include "list_of_function.h"
#include <stddef.h>
int16_t int16_t_abs(int16_t x) // 绝对值计算
{
    if (x < 0)
        x = -x;
    return x;
}

/*死区处理函数*/
int16_t Dead_Zone(int16_t Value,int16_t Limit)
{
	if(int16_t_abs(Value) < Limit)
		Value = 0;
	return Value;
}
/*数值大小限制函数*/
int16_t limit(int16_t x,int16_t max,int16_t min)
{
	if(x>max)
		x=max;
	else if(x<min)
		x=min;
	return x;
}
//数值限制
int32_t int32_limit(int32_t x, int32_t max, int32_t min)
{
    if (x > max)
        x = max;
    else if (x < min)
        x = min;
    return x;
}

int16_t int16_limit(int16_t x, int16_t max, int16_t min)
{
    if (x > max)
        x = max;
    else if (x < min)
        x = min;
    return x;
}

float float_limit(float x, float max, float min)
{
    if (x > max)
        x = max;
    else if (x < min)
        x = min;
    return x;
}

signed long limit_long(signed long x, signed long max, signed long min)
{
    if (x > max)
        x = max;
    else if (x < min)
        x = min;
    return x;
}

signed long long_abs(signed long x) // 绝对值计算
{
    if (x < 0)
        x = -x;
    return x;
}

float float_abs(float x) // 绝对值计算
{
    if (x < 0)
        x = -x;
    return x;
}

float invSqrt(float x) //平方根倒数速算法
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


/*找出最大值*/
int16_t RETURN_MAX(int16_t*Value,int16_t num)
{
	static int16_t MAX_I;
	int16_t MAX = int16_t_abs(Value[0]);
	for(MAX_I = 1;MAX_I < num;MAX_I ++)
	{
		if(int16_t_abs(Value[MAX_I]) > MAX)
			MAX = int16_t_abs(Value[MAX_I]);
	}
	return MAX;
}

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 运动控制斜坡函数（加速度限制）
 *
 * @param acceleration_control 加速度控制结构体
 * @param Input 控制量
 * @param Limit 限制加速度量
 * @return int16_t 处理量
 ************************** Dongguan-University of Technology -ACE***************************/
int16_t Motion_Acceleration_Control(Acceleration_Control_type_t *acceleration_control, int16_t Input, int16_t Limit)
{
    acceleration_control->Input = Input;
    acceleration_control->acc_limit = Limit;

    acceleration_control->acc_now = acceleration_control->Input - acceleration_control->Last_Input;
    if (int16_t_abs(acceleration_control->acc_now) > acceleration_control->acc_limit)
    {
        // output= Last_Input + acc_now /abs(acc_now) *acc_limit
        acceleration_control->Output = acceleration_control->Last_Input + acceleration_control->acc_now / int16_t_abs(acceleration_control->acc_now) * acceleration_control->acc_limit;
    }
    acceleration_control->Last_Input = acceleration_control->Input;

    return acceleration_control->Output;
}

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 一阶低通滤波计算
 *
 * @param first_order_filter_type 一阶低通滤波结构体
 * @param input
 * @return float
 ************************** Dongguan-University of Technology -ACE***************************/
float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    // out = input * num + (1-num)*last_input
    first_order_filter_type->out = first_order_filter_type->input * first_order_filter_type->num + (1 - first_order_filter_type->num) * first_order_filter_type->last_input;
    first_order_filter_type->last_input = first_order_filter_type->input;

    return first_order_filter_type->out;
}
/************************** Dongguan-University of Technology -ACE**************************
 * @brief 一阶低通滤波初始化
 *
 * @param first_order_filter_type 一阶低通滤波结构体
 * @param num
 ************************** Dongguan-University of Technology -ACE***************************/
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 num)
{
    if (first_order_filter_type == NULL)
    {
        return;
    }

    first_order_filter_type->input = 0;
    first_order_filter_type->last_input = 0;
    first_order_filter_type->num = num;
    first_order_filter_type->out = 0;
}

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 滑动均值滤波参数初始化(浮点型)
 *
 * @param mean_filter 滤波对象结构体
 ************************** Dongguan-University of Technology -ACE***************************/
void Sliding_Mean_Filter_Init(sliding_mean_filter_type_t *mean_filter)
{
    mean_filter->count_num = 0;
    for (uint8_t i = 0; i < 20; i++)
    {
        mean_filter->FIFO[i] = 0.0f;
    }
    mean_filter->Input = 0.0f;
    mean_filter->Output = 0.0f;
    mean_filter->Sum = 0.0f;
    mean_filter->sum_flag = 0;
}
/************************** Dongguan-University of Technology -ACE**************************
 * @brief 滑动均值滤波（浮点型）
 *        ------抑制小幅度高频噪声
 *
 * @param mean_filter 滤波对象结构体
 * @param Input 更新值
 * @param num 均值数量
 * @return 滑动滤波输出值（250次）
 ************************** Dongguan-University of Technology -ACE***************************/
float Sliding_Mean_Filter(sliding_mean_filter_type_t *mean_filter, float Input, int num)
{
    //更新数据
    mean_filter->Input = Input;
    mean_filter->FIFO[mean_filter->count_num] = mean_filter->Input;
    mean_filter->count_num++;

    if (mean_filter->count_num == num)
    {
        mean_filter->count_num = 0;
        mean_filter->sum_flag = 1;
    }
    //求和
    if (mean_filter->sum_flag == 1)
    {
        for (int count = 0; count < num; count++)
        {
            mean_filter->Sum += mean_filter->FIFO[count];
        }
    }
    //均值
    mean_filter->Output = mean_filter->Sum / num;
    mean_filter->Sum = 0;

    return mean_filter->Output;
}

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 正负循环限制（16位）
 *        将输入值限制在 +-限制幅度 的范围内
 * @param num 输入值
 * @param limit_num 限制幅度(正数)
 * @return int16_t 限幅输出值
 ************************** Dongguan-University of Technology -ACE***************************/
int16_t Loop_Restriction_Int16(int16_t num, int16_t limit_num)
{
    if (int16_t_abs(num) > limit_num)
    {
        if (num >= 0)
            num -= limit_num;
        else
            num += limit_num;
    }
    return num;
}
/************************** Dongguan-University of Technology -ACE**************************
 * @brief 正负循环限制（float）
 *        将输入值限制在 +-限制幅度 的范围内
 * @param num 输入值
 * @param limit_num 限制幅度(正数)
 * @return float 限幅输出值
 ************************** Dongguan-University of Technology -ACE***************************/
float Loop_Restriction_Float(float num, float limit_num)
{
    if (float_abs(num) > limit_num)
    {
        if (num >= 0)
            num -= limit_num;
        else
            num += limit_num;
    }
    return num;
}

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 循环限幅fp32
 * 
 * @param Input 
 * @param minValue 
 * @param maxValue 
 * @return float 
 ************************** Dongguan-University of Technology -ACE***************************/
float loop_fp32_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//void Data_Acceleration_Control(float angle)
//{
//    static int16_t last_num=0;
//    int16_t temp;

//    
//}




// sin函数查表法
float sin_calculate(float angle)
{
    float sin_angle;

    if (angle >= 0.0f && angle < 90.0f)
        sin_angle = (Trigonometric_Functions[(int)(float_abs(angle) * 10.0f)] / 100.0f);
    else if (angle >= 90.0f && angle < 180.f)
        sin_angle = (Trigonometric_Functions[(int)(float_abs(180.0f - angle) * 10.0f)] / 100.0f);
    else if (angle >= -180.0f && angle < -90.f)
        sin_angle = -(Trigonometric_Functions[(int)(float_abs(180.0f + angle) * 10.0f)] / 100.0f);
    else if (angle >= -90.0f && angle < 0.f)
        sin_angle = -(Trigonometric_Functions[(int)(float_abs(180.0f - (180.0f + angle)) * 10.0f)] / 100.0f);
    else if (angle == 180.f)
        sin_angle = 0.0f;

    return sin_angle;
}

// cos函数查表法
float cos_calculate(float angle)
{
    float cos_angle;

    angle = float_abs(angle);

    if (angle >= 0.0f && angle < 90.0f)
        cos_angle = (Trigonometric_Functions[(int)(float_abs(90.0f - angle) * 10.0f)] / 100.0f);
    else if (angle >= 90.0f && angle < 180.f)
        cos_angle = -(Trigonometric_Functions[(int)(float_abs(angle - 90.0f) * 10.0f)] / 100.0f);
    else if (angle == 180.f)
        cos_angle = -1.0f;

    return cos_angle;
}
