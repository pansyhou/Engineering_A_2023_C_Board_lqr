/************************** Dongguan-University of Technology -ACE**************************
 * @file bsp_Motor_Encoder.c
 * @brief 
 * @author pansyhou侯文辉 (1677195845lyb@gmail.com)
 * @version 1.0
 * @date 2022-11-24
 * 
 * 
 * @history
 * <table>
 * Date       Version Author Description
 * 2022-11-24   1.0   侯文辉     
 * @verbatim 
 * ==============================================================================
 *  已经把常用的速度、位置、加速度、多圈编码、堵转检测巴拉巴拉的混进去了。
 *  在结构体能用的部分：
 *      Encode_Record_Val多圈编码器记录值
 *      Radian转过的弧度
 *      User_Radian弧度自定义，自己调PID的时候可以设置
 *      Speed速度
 *      AccSpeed加速度
 *      position位置
 *      State电机状态
 *
 *  使用方法：
 *      将CAN_DATA_Encoder_Deal扔CAN接收部分
 *      在初始化阶段加入Encoder_Init
 *      done
 *  注意事项：
 *      改减速比的电机只能直接硬改结构体的gear_Ratio
 *      Max_Block_Angle_Num、Max_Block_Num这两个都是调整堵转灵敏度的，需要自己在初始化阶段调整
 *      CAN_ENCODER_NUM宏定义目前设置为10，方便工程有很多电机和编码器
 *
 *  
 * ==============================================================================
 * @endverbatim
************************** Dongguan-University of Technology -ACE***************************/
#include <string.h>
#include "bsp_Motor_Encoder.h"
//版本1是家华写的，用的是最原始的临界值判断的方案，但是感觉没有差值好使
//版本2是差值+溢出 计算的多圈编码器，频率是够用的
#define encoder_verson 2    

Encoder_t CAN_Encoder[CAN_ENCODER_NUM];

/**
 * 编码器初始化
 * @param Encoder 编码器指针
 * @param Encoder_Type 编码器种类
 * @return HAL_StatusTypeDef
 */

/**
 * 编码器初始化，返回指针
 * @param Encoder_Type 编码器种类
 * @param ch encoder编号（从1开始）
 * @return
 */
Encoder_t *Encoder_Init(Encoder_Type_e Encoder_Type , uint8_t ch){

    if(ch>=CAN_ENCODER_NUM)return NULL;

    Encoder_t *Encoder=&CAN_Encoder[ch-1];
    EncoderValZero(Encoder);
    Encoder->Encoder_Type = Encoder_Type;
    Encoder->State = NORM;      //状态

    switch (Encoder_Type) {
        case M3508: {
            Encoder->lap_encoder = 8192;//编码器单圈码盘值
            Encoder->gear_Ratio = 19;   //3508减速比
            Encoder->Max_Block_Angle_Num = 10;
            Encoder->Max_Block_Num = 256;
        }break;

        case GM6020:{
            Encoder->lap_encoder = 8192;//编码器单圈码盘值
            Encoder->gear_Ratio = 1;   //减速比
        }break;

        case Rotary: {
            Encoder->lap_encoder = 8192;//编码器单圈码盘值
            Encoder->gear_Ratio = 1;   //减速比
        }break;
    }

    return Encoder;
}

/**
 * CAN编码器处理函数
 * @param position 位置
 * @param speed 速度
 * @param Encoder_Num 编码器编号（从1开始
 */
void CAN_DATA_Encoder_Deal(int16_t position, int16_t speed, uint8_t Encoder_Num) {
    //返回指针
    Encoder_t *Encoder = &CAN_Encoder[Encoder_Num-1];
    Encoder->position = position;    //未处理的Can原始码盘

    /*速度处理*/
    Encoder->Speed[1] = speed;
    //加速度计算
    Encoder->AccSpeed = Encoder->Speed[1] - Encoder->Speed[0];
    Encoder->Speed[0] = Encoder->Speed[1];
    //第一次进入初始化position
    if (Encoder->Init_Flag == 0) {
        Encoder->Init_Flag = 1;
        Encoder->last_position = Encoder->position;
    }

#if encoder_verson == 1
    /*码盘状态判断*/
    if (CanData <= 0 || CanData > 8192) {
        Encoder->State = WRONG;
    } else {
        Encoder->State = NORM;
    }

    /*码盘处理*/
    /*零界点判断*/
    if (CanData < 7000 && CanData > 2000) {
        /*码盘值在一小圈内*/
        Encoder->Ahead[0] = 0;
        Encoder->Ahead[1] = 0;
        Encoder->Back[0] = 0;
        Encoder->Back[1] = 0;
    } else if (CanData > 7000) {
        /*码盘值正向接近一小圈*/
        Encoder->Ahead[0] = 1;
    } else if (CanData < 2000) {
        /*码盘值反向接近一小圈*/
        Encoder->Back[0] = 1;
    }

    /*越过零界点判断*/
    if (Encoder->Ahead[0] == 1 && CanData < 2000) {
        /*码盘值正向越过一小圈*/
        Encoder->Ahead[1] = 1;
    } else if (Encoder->Back[0] == 1 && CanData > 7000) {
        /*码盘值反向越过一小圈*/
        Encoder->Back[1] = 1;
    }

    /*零界点处理*/
    if (Encoder->Ahead[0] == 1 && Encoder->Ahead[1] == 1) {
        Encoder->Radio_Circle++;            //累加一小圈
        Encoder->Encode_Record_Val += 8192; //累加码盘值
        Encoder->Ahead[0] = 0;                //标志位清零
        Encoder->Ahead[1] = 0;                //标志位清零
    } else if (Encoder->Back[0] == 1 && Encoder->Back[1] == 1) {
        Encoder->Radio_Circle--;            //累减一小圈
        Encoder->Encode_Record_Val -= 8192; //累减码盘值
        Encoder->Back[0] = 0;                //标志位清零
        Encoder->Back[1] = 0;                //标志位清零
    }

    if (Encoder->Radio_Circle > 0) {
        Encoder->Encode_Actual_Val = Encoder->Radio_Circle * 8192 + CanData; //码盘一大圈内的真实码盘值
    } else if (Encoder->Radio_Circle < 0) {
        Encoder->Encode_Actual_Val = Encoder->Radio_Circle * 8192 - 8192 + CanData; //码盘一大圈内的真实码盘值
    } else if (Encoder->Radio_Circle == 0) {
        if (Encoder->Encode_Record_Val > 8192) //反向跨越
        {
            Encoder->Encode_Actual_Val = CanData - 8192 + Encoder->Radio_Circle * 8192;
        } else if (Encoder->Encode_Record_Val <= 8192) //正向跨越
        {
            Encoder->Encode_Actual_Val = CanData;
        }
    }

    /*电机一大圈处理*/
    /*电机正向转过一大圈*/
    if (Encoder->Encode_Record_Val > (8192 * Encoder->gear_Ratio)) {
        Encoder->Encode_Record_Val = CanData; //码盘记录值为当前值
        Encoder->Actual_Circle++;              //电机实际圈数累加
        Encoder->Radio_Circle = 0;              //清零小圈
    }
        /*电机反向转过一大圈*/
    else if (Encoder->Encode_Record_Val < 0) {
        Encoder->Encode_Record_Val = 8192 * (Encoder->gear_Ratio - 1) + CanData; //码盘记录值复位
        Encoder->Actual_Circle--;                                   //码盘一大圈累减
        Encoder->Radio_Circle = 0;                                   //清零小圈
    } else {
        Encoder->Radian = (360.0 / (Encoder->gear_Ratio * 8192) * (Encoder->Encode_Actual_Val)); //一大圈角度
    }

    if (Encoder->State == WRONG) {
    } else if (Encoder->State == BLOCK) {
    }
#endif

#if encoder_verson == 2

    //多圈码盘值，不做360度后归零处理
    Encoder->Encode_Record_Val += angle_limiting_int16(Encoder->position - Encoder->last_position,Encoder->lap_encoder); //差值累加
    Encoder->Total_Radian = (float)((Encoder->Encode_Record_Val * Encoder->gear_Ratio) / Encoder->lap_encoder);
    //    用于360度的限幅（好像是超过360归零的，但是我不需要）
    //    Encoder->Encode_Actual_Val = check_codevalue(Encoder->Encode_Actual_Val , radio, Encoder->lap_encoder);             //过临界值复位码盘值
    Encoder->last_position = Encoder->position;

    //电机堵转检测
    Encoder->State=Block_Detect(Encoder->position - Encoder->last_position, Encoder);


#endif

}




/**
 * 码盘值数值清零处理
 * @param Encoder
 */
void EncoderValZero(Encoder_t *Encoder) {
    memset((void *)Encoder, 0x0, sizeof(Encoder_t));
}


/**
 * 堵转检测 TODO：还没测试，还没有考虑解堵转后
 * @param error 上一次和这次位置的差值
 * @param Encoder
 * @return
 */
STATE_e Block_Detect(int16_t error, Encoder_t *Encoder ) {
    //其他类型的编码器比如拉线，就不用考虑堵转
    if(Encoder->Encoder_Type>=Rotary)return NORM;

    //绝对值小于Max_Block_Angle_Num堵转最大角度容忍值时计数器狂加
    if (error > Encoder->Max_Block_Angle_Num) {
        Encoder->Block_Count++;
    } else if (error < -Encoder->Max_Block_Angle_Num) {
        Encoder->Block_Count++;
    } else {
        Encoder->Block_Count = 0;
    }

    //最后判断是不是堵转咯
    if (Encoder->Block_Count > Encoder->Max_Block_Num)
        return BLOCK;
    else
        return NORM;

}

//临角处理16位（对应角度正值）
int16_t angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder) {
    //|当前值 - 上一次值| > 编码器最大值/2 时说明向上溢出
    if (Angl_Err < -(lap_encoder / 2))
    {
        Angl_Err += (lap_encoder - 1);
    }
    if (Angl_Err > (lap_encoder / 2)) {
        Angl_Err -= (lap_encoder - 1);
    }
    return Angl_Err;
}

//过临界值复位码盘值 （限制于360度的码盘值循环 DJI电机）
int32_t check_codevalue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder) {
    if (value > (gear_Ratio * lap_encoder) / 2) {
        value = value - (gear_Ratio * lap_encoder);
    }
    if (value < (-(gear_Ratio * lap_encoder) / 2)) {
        value = (gear_Ratio * lap_encoder) - value;
    }
    return value;
}

