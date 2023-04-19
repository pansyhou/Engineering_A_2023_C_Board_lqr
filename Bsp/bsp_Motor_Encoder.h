#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "main.h"
#include "struct_typedef.h"

#define CAN_ENCODER_NUM 10

/*数据状态枚举*/
typedef enum {
    NORM,  //正常
    BLOCK, //堵转
    WRONG  //异常
} STATE_e;

typedef enum {
    M3508,//如果有改减速比的，在初始化之后自己硬改
    GM6020,
    Rotary//工程上用的拉线位移传感器
} Encoder_Type_e;


/*CAN数据处理-码盘处理*/
typedef __packed struct {
    //有很多都是家华的encoder的内容，先留着，以后说不定用上
    uint8_t Ahead[2];//0用于第一次判断是否>7000准备越过8192接近正向小一圈，1用于判断是否已经正向越过小一圈
    uint8_t Back[2];//0用于第一次判断是否<2000准备越过8192接近反向小一圈，1用于判断是否已经反向越过小一圈
    int16_t Radio_Circle;    //小圈数
    int16_t Actual_Circle;    //大圈数
    int32_t Encode_Record_Val;//累积码盘值(在小圈内)
    int32_t Encode_Actual_Val;//真实码盘值(一大圈)

    float Lock_Radian;

    float Radian;    //弧度
    float User_Radian;
    float Total_Radian;

    uint8_t Init_Flag;
    int16_t Speed[2];    //0为旧速度，1为新速度
    int16_t AccSpeed;    //加速度
    int16_t position;    //未处理的Can原始码盘
    int16_t last_position;    //未处理的上次的Can原始码盘
    int16_t lap_encoder;      //编码器单圈码盘值（8192=12bit）
    Encoder_Type_e Encoder_Type;//编码器种类
    STATE_e State;              //电机状态
    int16_t gear_Ratio;         //减速比
    int16_t Max_Block_Angle_Num;      //堵转最大角度容忍值，可以调整这个来控制堵转灵敏度
    int16_t Max_Block_Num;      //堵转计数器最大值
    int32_t Block_Count;
} Encoder_t;

STATE_e Block_Detect(int16_t error, Encoder_t *Encoder );

Encoder_t *Encoder_Init(Encoder_Type_e Encoder_Type,uint8_t ch);


/*CAN返回码盘值处理*/
void CAN_DATA_Encoder_Deal( int16_t position, int16_t speed,  uint8_t Encoder_Num);

/*码盘值数值清零处理*/
void EncoderValZero(Encoder_t *Encoder);

int16_t angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder);

int32_t check_codevalue(int32_t value, int16_t gear_Ratio, int16_t lap_encoder);

#endif
