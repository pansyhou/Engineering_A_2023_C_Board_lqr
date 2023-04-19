#include "pid.h"
#include "SYSInit.h"

/**
 * @brief 一个PI控制器
 *
 * @param motor
 * @param set_Speed
 * @param current_Speed
 * @return fp32
 */
PIDTypeDef PID_STOP;

// /**
//  * @brief 刹车结构体初始化
//  *
//  * @return fp32
//  */
// fp32 PID_STOP_Init(void)
// {
//     PID_STOP.kp = 3;
//     PID_STOP.ki = 0;
//     PID_STOP.kd = 3;
//     PID_STOP.H_Limited = 2000;
//     PID_STOP.L_Limited = -2000;
//     PID_STOP.SetValues = 0;
// 	return 1;
// }



/************************** Dongguan-University of Technology -ACE**************************
 * @brief PID初始化
 * 
 * @param PID_Init_t PID结构体
 * @param kp 
 * @param ki 
 * @param kd 
 * @param Output_Limit 输出限制
 * @param max_iout I项限制
************************** Dongguan-University of Technology -ACE***************************/
void PID_Init(PIDTypeDef *PID_Init_t, fp32 kp, fp32 ki, fp32 kd, float Output_Limit, fp32 max_iout)
{
    PID_Init_t->kp = kp;
    PID_Init_t->ki = ki;
    PID_Init_t->kd = kd;
    PID_Init_t->max_output=Output_Limit;
    PID_Init_t->max_iout=max_iout;
}


/************************** Dongguan-University of Technology -ACE**************************
 * @brief 普普通通PID计算，记得更新currentValue 和 SetValues
 * 
 * @param pid 
 * @return fp32 输出
************************** Dongguan-University of Technology -ACE***************************/
fp32 PIDcal(PIDTypeDef *pid)
{
    //差值，0为旧的，1是新的
    pid->error[1] = pid->SetValues - pid->CurrentValues;

    pid->P_Out = (pid->kp * pid->error[1]);
    pid->I_out += (pid->ki * pid->error[1]);
    pid->D_Out = (pid->kd * (pid->error[1] - pid->error[0]));

    //积分微分限幅
    pid->I_out = PIDlimit(pid->I_out, pid->max_iout, -pid->max_iout);
    pid->D_Out = PIDlimit(pid->D_Out, pid->max_output, -pid->max_output);

    pid->Output = pid->P_Out + pid->I_out + pid->D_Out;
    //输出限幅
    pid->Output = PIDlimit(pid->Output, pid->max_output, -pid->max_output);

    pid->error[0] = pid->error[1];
    pid->Last_CurrentValues = pid->CurrentValues;

    return pid->Output;
}



// /**
//  * @brief 位置速度双环pid，包含了防跑飞功能？不要用
//  *
//  * @param SpeedPid 速度环PID结构体指针
//  * @param PoisionPid 位置环（外环）PID指针
//  * @param current_speed 当前速度
//  * @param current_poision 当前位置
//  * @param set_poision 设置位置
//  * @return fp32 电流值
//  */
// fp32 PID_Position_speed_Control(PIDTypeDef *SpeedPid, PIDTypeDef *PoisionPid, fp32 current_speed, fp32 current_poision, fp32 set_poision)
// {
//         PoisionPid->SetValues = set_poision;

//         PoisionPid->CurrentValues = current_poision;

//         SpeedPid->CurrentValues = current_speed;
//         if (PoisionPid->Last_CurrentValues - PoisionPid->CurrentValues > 3000 || PoisionPid->Last_CurrentValues - PoisionPid->CurrentValues < -3000)
//             PoisionPid->Circl_Cut++;
//         SpeedPid->SetValues = PIDcal(PoisionPid);
//         fp32 output = PIDcal(SpeedPid);
//         return output;

//     else if (PoisionPid->Circl_Cut >= 2 && PoisionPid->Circl_Cut < 10) //防跑飞刹车时间在这调
//     {
//         PID_STOP.CurrentValues = current_speed;
//         PoisionPid->Circl_Cut++;
//         return PIDcal(&PID_STOP);
//     }
//     else
//     {
//         PID_STOP.CurrentValues = current_speed;
//         PoisionPid->Circl_Cut = 0;
//         return PIDcal(&PID_STOP);
//     }
// }

fp32 PID_Position_speed_Deal(PIDTypeDef *SpeedPid, PIDTypeDef *PoisionPid)
{

    SpeedPid->SetValues = PIDcal(PoisionPid);
    fp32 output = PIDcal(SpeedPid);
    return output;
}

fp32 PIDlimit(fp32 object, fp32 H_Limitation, fp32 L_Limitation)
{
    if (object > H_Limitation)
        object = H_Limitation;
    else if (object < L_Limitation)
        object = L_Limitation;
    return object;
}

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 步进式PID控制设定值步进处理函数
 *  当设定值发生较大的突变时，很容易产生超调而使系统不稳定。
 *  为了解决这种阶跃变化造成的不利影响,采用步进式PID。
 *  https://blog.csdn.net/foxclever/article/details/81151898
 * @param vPID
 * @param sp setposition
 * @return float
 ************************** Dongguan-University of Technology -ACE***************************/
//float step_in_processing(PIDTypeDef *vPID, float sp)
//{
//    // float stepIn = (vPID->maximum - vPID->minimum) * 0.1f + vPID->minimum;
//    float stepIn = vPID->stepIn;
//    float kFactor = 0.0f;

//    if (fabs(vPID->SetValues - sp) <= stepIn)
//    {
//        vPID->SetValues = sp;
//    }
//    else
//    {
//        if (vPID->SetValues - sp > 0)
//        {
//            kFactor = -1.0f;
//        }
//        else if (vPID->SetValues - sp < 0)
//        {
//            kFactor = 1.0f;
//        }
//        else
//        {
//            kFactor = 0.0f;
//        }

//        vPID->SetValues = vPID->SetValues + kFactor * stepIn;
//    }

//    return vPID->SetValues;
//}




int32_t PID_regulator(PIDTypeDef *vPID, float actualValue)
{
    // error
    vPID->error[1] = vPID->SetValues - vPID->CurrentValues;

    if (vPID->Output > vPID->max_output)
    {
        if (vPID->error[1] <= 0)
        {
            vPID->Ierror += vPID->error[1];
        }
    }
    else if (vPID->Output < -vPID->max_output)
    {
        if (vPID->error[1] >= 0)
        {
            vPID->Ierror += vPID->error[1];
        }
    }
    else
    {
        vPID->Ierror += vPID->error[1];
    }

    //积分限幅
    vPID->Ierror=int32_limit(vPID->Ierror,vPID->max_iout,-vPID->max_iout);

    vPID->Derror[0]=vPID->error[1]-vPID->error[0];

    vPID->P_Out = (vPID->kp * vPID->error[1]);
	vPID->I_out = (vPID->ki * vPID->Ierror);
	vPID->D_Out = (vPID->kd * vPID->Derror[0]);

    vPID->Output=vPID->P_Out+vPID->I_out+vPID->D_Out;

    //输出限幅
    vPID->Output=PIDlimit(vPID->Output, vPID->max_output, -vPID->max_output);
    //记录error
    vPID->error[0]=vPID->error[1];
    
    return vPID->Output;
}

