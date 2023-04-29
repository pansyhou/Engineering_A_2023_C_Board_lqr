/************************** Dongguan-University of Technology -ACE**************************
 * @file GimbalMotor.c
 * @brief
 * @author pansyhou侯文辉 (1677195845lyb@gmail.com)
 * @version 1.0
 * @date 2022-07-16
 *
 *
 * @history 
 * <table>
 * Date       Version Author Description
 * 2022-07-16   1.0   侯文辉 彭淐
 * @verbatim
 * ==============================================================================
 * 虽然他叫gimbal，但是，他只是说明，这是云台板上连的电机
 * 文件内容是，每个模块对应的电机工作控制
 * 1.抬升驱动
 * 2.夹爪翻转（翻转夹子，把矿扔到储矿位置
 * 3.夹爪抓取
 * 4.矿翻转（将矿翻转
 * 5.伸缩驱动
 * ==============================================================================
 * @endverbatim
 ************************** Dongguan-University of Technology -ACE***************************/
#include "GimbalMotor.h"
#include "bsp_buzzer.h"
#include "bsp_can.h"
#include "bsp_pwm.h"
#include "pid.h"
#include "imu_task.h"
#include "SYSInit.h"
#include "arm_math.h"
Lift_UP_t Lift_t;
Three_D_Arm_t TD_t;
Clip_Module_t Clip_t;
Grasp_t Grasp;
can_std_msg Gimbal_Can_msg;
	const INS_t *IMU;

void MotorDataDeal(CAN_RxHeaderTypeDef *header, uint8_t *data);
Grasp_t *Return_Grasp_t_Pointer(void) {
    return &Grasp;
}


/**
 *  这个函数用于CAN中断调用的数据处理函数，已经在CAN_Drive注册了
 * @param header
 * @param data
 */
void MotorDataDeal(CAN_RxHeaderTypeDef *header, uint8_t *data) {
    /**
     * 资源分配愿景
     * 机械臂部分roll、pitch、traverse三个要占用0x200的
     * 抬升占两个0x200，前伸占一个0x200
     * 0x200资源总共占6个，0x202是寄了的，刚好剩一个出来给多一个自由度
     * 最后就是机械臂的yaw是6020,让他跑去2ff去了	
     * 那就抬升和前伸占一个can通道，分配到1/3/4号
     * 机械臂占一个通道，5/6/7/8 多出来一个给未来的一个自由度，虽然说不知道会不会换步进电机
     */
     /*
      * 重新分配
      * 1-2号给前伸
      * 3-4号 最后自由度的两个2006
      * 5 pitch改过减速比3508
      * 6 roll3508
      * 7
      * 8 横移2006
      * 9 yaw6020
      */
    switch (header->StdId) {
            //    1-2 前伸
        case CAN_M3508_MOTOR1_ID://
        {
            CAN_DATA_Encoder_Deal((int16_t) ((data[0] << 8) + (data[1])),//电机位置
                                  (int16_t) ((data[2] << 8) + (data[3])),//电机速度
                                  1);
            TD_t.Forward_Motor[0].anper = (int16_t) ((data[4] << 8) + (data[5]));//当前电机点流
            TD_t.Forward_Motor[0].temp = (int16_t) (data[6]);                    //当前电机温度
        } break;

//        case CAN_M3508_MOTOR3_ID://抬升电机
//        {
//            CAN_DATA_Encoder_Deal((int16_t) ((data[0] << 8) + (data[1])),//电机位置
//                                  (int16_t) ((data[2] << 8) + (data[3])),//电机速度
//                                  3);
//            Lift_t.Lift_Motor[1].anper = (int16_t) ((data[4] << 8) + (data[5]));//当前电机点流
//            Lift_t.Lift_Motor[1].temp = (int16_t) (data[6]);                    //当前电机温度
//        } break;
//        case CAN_M3508_MOTOR3_ID://最后自由度的2006
//        {
//            CAN_DATA_Encoder_Deal((int16_t) ((data[0] << 8) + (data[1])),//电机位置
//                                  (int16_t) ((data[2] << 8) + (data[3])),//电机速度
//                                  3);
//            TD_t.Last_Joint[0].torque = (int16_t) ((data[4] << 8) + (data[5]));//当前电机转矩
//        } break;
//        case CAN_M3508_MOTOR4_ID://最后自由度的2006
//        {
//            CAN_DATA_Encoder_Deal((int16_t) ((data[0] << 8) + (data[1])),//电机位置
//                                  (int16_t) ((data[2] << 8) + (data[3])),//电机速度
//                                  4);
//            TD_t.Last_Joint[1].torque = (int16_t) ((data[4] << 8) + (data[5]));//当前电机转矩
//        } break;
            //6020的yaw占用了，先关
          case CAN_M3508_MOTOR5_ID://pitch
          {
            CAN_DATA_Encoder_Deal((int16_t)((data[0] << 8) + (data[1])), //电机位置
                                  (int16_t)((data[2] << 8) + (data[3])), //电机速度
                                  5);
            TD_t.Pitch_Motor.anper = (int16_t)((data[4] << 8) + (data[5]));//当前电机点流
            TD_t.Pitch_Motor.temp = (int16_t)(data[6]);//当前电机温度
          }break;
        case CAN_M3508_MOTOR6_ID://roll
        {
            CAN_DATA_Encoder_Deal((int16_t) ((data[0] << 8) + (data[1])),//电机位置
                                  (int16_t) ((data[2] << 8) + (data[3])),//电机速度
                                  6);
            TD_t.Roll_Motor.anper = (int16_t) ((data[4] << 8) + (data[5]));//当前电机点流
            TD_t.Roll_Motor.temp = (int16_t) (data[6]);                    //当前电机温度
        } break;
            //预留位
            //  case CAN_M3508_MOTOR7_ID:
            //  {
            //    CAN_DATA_Encoder_Deal((int16_t)((data[0] << 8) + (data[1])), //电机位置
            //                          (int16_t)((data[2] << 8) + (data[3])), //电机速度
            //                          7);
            //  }break;

            //机械臂横移2006
//        case CAN_M3508_MOTOR8_ID: {
//            CAN_DATA_Encoder_Deal((int16_t) ((data[0] << 8) + (data[1])),//电机位置
//                                  (int16_t) ((data[2] << 8) + (data[3])),//电机速度
//                                  8);
//            TD_t.Traverse_Motor.torque = (int16_t) ((data[4] << 8) + (data[5]));//当前电机转矩
//        } break;
      case CAN_GM6020_MOTOR5_ID:
      {
        CAN_DATA_Encoder_Deal((int16_t)((data[0] << 8) + (data[1])), //电机位置
                              (int16_t)((data[2] << 8) + (data[3])), //电机速度
                              9);
        TD_t.Yaw_Motor.torque = (int16_t)((data[4] << 8) + (data[5]));//当前电机转矩
        TD_t.Yaw_Motor.temp = (int16_t)(data[6]);//当前电机温度
      } break;

    default:
        break;
    }
}


void Gimbal_Motor_Init(void) {
    Grasp.Lift_t = &Lift_t;
		IMU = get_imu_control_point();
	
    float Spid[9][3] =
            {
                    {Forword_Motor1_Spid_P, Forword_Motor1_Spid_I, Forword_Motor1_Spid_D},
                    {Forword_Motor2_Spid_P, Forword_Motor2_Spid_I, Forword_Motor2_Spid_D},
                    {Pitch_3508_Spid_P, Pitch_3508_Spid_I, Pitch_3508_Spid_D},
                    {Roll_Spid_P, Roll_Spid_I, Roll_Spid_D},
                    {Yaw_6020_Spid_P, Yaw_6020_Spid_I, Yaw_6020_Spid_D},
                    {Tranverse_2006_Spid_P, Tranverse_2006_Spid_I, Tranverse_2006_Spid_D},
                    {LastJoint_2006_Spid_P, LastJoint_2006_Spid_I, LastJoint_2006_Spid_D}
                    
            };

    float Ppid[9][3] =
            {
                    {Forword_Motor1_Ppid_P, Forword_Motor1_Ppid_I, Forword_Motor1_Ppid_D},
                    {Forword_Motor2_Ppid_P, Forword_Motor2_Ppid_I, Forword_Motor2_Ppid_D},
                    {Pitch_3508_Ppid_P, Pitch_3508_Ppid_I, Pitch_3508_Ppid_D},
                    {Roll_Ppid_P, Roll_Ppid_I, Roll_Ppid_D},
                    {Yaw_6020_Ppid_P, Yaw_6020_Ppid_I, Yaw_6020_Ppid_D},
                    {Tranverse_2006_Ppid_P, Tranverse_2006_Ppid_I, Tranverse_2006_Ppid_D},
                    {LastJoint_2006_Ppid_P, LastJoint_2006_Ppid_I, LastJoint_2006_Ppid_D}
            };

    /********************抬升部分初始化********************/

    //Lift_Motor电机结构体赋值
    //初始化多圈编码器 Initialize the multiturn encoder
//    Lift_t.Lift_Motor[0].Encoder = Encoder_Init(M3508, 1);//一号是前进方向的？
//    Lift_t.Lift_Motor[1].Encoder = Encoder_Init(M3508, 3);

    // PID初始化 powered by ECF
    PidInit(&TD_t.Forward_Motor[0].SPID, Spid[0][0], Spid[0][1], Spid[0][2], Integral_Limit | Output_Limit);
    PidInit(&TD_t.Forward_Motor[0].PPID, Ppid[0][0], Ppid[0][1], Ppid[0][2], Integral_Limit | Output_Limit);

    PidInit(&TD_t.Forward_Motor[1].SPID, Spid[1][0], Spid[1][1], Spid[1][2], Integral_Limit | Output_Limit);
    PidInit(&TD_t.Forward_Motor[1].PPID, Ppid[1][0], Ppid[1][1], Ppid[1][2], Integral_Limit | Output_Limit);

    //积分和输出限幅
    PidInitMode(&TD_t.Forward_Motor[0].SPID, Integral_Limit, 200, 200);
    PidInitMode(&TD_t.Forward_Motor[0].SPID, Output_Limit, 6000, 0);
    PidInitMode(&TD_t.Forward_Motor[0].PPID, Integral_Limit, 200, 200);
    PidInitMode(&TD_t.Forward_Motor[0].PPID, Output_Limit, 6000, 0);

    PidInitMode(&TD_t.Forward_Motor[1].SPID, Integral_Limit, 200, 200);
    PidInitMode(&TD_t.Forward_Motor[1].SPID, Output_Limit, 6000, 0);
    PidInitMode(&TD_t.Forward_Motor[1].PPID, Integral_Limit, 200, 200);
    PidInitMode(&TD_t.Forward_Motor[1].PPID, Output_Limit, 6000, 0);

    /*******************机械臂部分初始化*******************/

    // 绑定机械臂结构体到大结构体上
    Grasp.TD_t = &TD_t;
    // 初始化吸盘状态
    TD_t.Suker_state = 0;

    //TODO:
    //init arm length
    TD_t.l1 = ARM_L1_LENTH;
    TD_t.l2 = ARM_L2_LENTH;
    TD_t.l3 = INIT_ARM_L3_LENTH;
    //init arm angle
    TD_t.Pitch1_Angle = 0.0f;
    TD_t.Pitch2_Angle = 0.0f;
    TD_t.l3ToHorizontalPlane_Angle = 0.0f;


    //初始化多圈编码器 Initialize the multiturn encoder
    // get each encoder
    TD_t.Pitch_Motor.Encoder = Encoder_Init(M3508_EngineeringPitch, 5);

    TD_t.Roll_Motor.Encoder = Encoder_Init(M3508, 6);
    //注意，为了最大利用CAN总线，让6020 ID设置在5号之后是最佳的
    TD_t.Yaw_Motor.Encoder = Encoder_Init(GM6020, 9);
//    //初始化横移的2006电机PID ,8号电调 ,只做速度环
//    TD_t.Traverse_Motor.Encoder = Encoder_Init(M2006, 8);
//
//    TD_t.Last_Joint[0].Encoder = Encoder_Init(M3508_EngineeringPitch, 3);
//    TD_t.Last_Joint[1].Encoder = Encoder_Init(M3508_EngineeringPitch, 4);

    TD_t.Forward_Motor[0].Encoder = Encoder_Init(M3508, 1);
    TD_t.Forward_Motor[1].Encoder = Encoder_Init(M3508, 2);


    // each motor speed pid and position pid init
    PidInit(&TD_t.Traverse_Motor.SPID, Spid[5][0], Spid[5][1], Spid[5][2], Integral_Limit | Output_Limit);
    PidInitMode(&TD_t.Traverse_Motor.SPID, Integral_Limit, 200, 200);
    PidInitMode(&TD_t.Traverse_Motor.SPID, Output_Limit, 10000, 0);

    //pitch motor speed pid init
    PidInit(&TD_t.Pitch_Motor.SPID, Spid[2][0], Spid[2][1], Spid[2][2], Integral_Limit | Output_Limit);
    PidInitMode(&TD_t.Pitch_Motor.SPID, Integral_Limit, 200, 200);
    PidInitMode(&TD_t.Pitch_Motor.SPID, Output_Limit, 9000, 0);
    //pitch motor position pid init
    PidInit(&TD_t.Pitch_Motor.PPID, Ppid[2][0], Ppid[2][1], Ppid[2][2], Integral_Limit | Output_Limit);
    PidInitMode(&TD_t.Pitch_Motor.PPID, Integral_Limit, 200, 200);
    PidInitMode(&TD_t.Pitch_Motor.PPID, Output_Limit, 9000, 0);

    //roll motor speed pid init
    PidInit(&TD_t.Roll_Motor.SPID, Spid[3][0], Spid[3][1], Spid[3][2], Integral_Limit | Output_Limit);
    PidInitMode(&TD_t.Roll_Motor.SPID, Integral_Limit, 200, 200);
    PidInitMode(&TD_t.Roll_Motor.SPID, Output_Limit, 5000, 0);
    //roll motor position pid init
    PidInit(&TD_t.Roll_Motor.PPID, Ppid[3][0], Ppid[3][1], Ppid[3][2], Integral_Limit | Output_Limit);
    PidInitMode(&TD_t.Roll_Motor.PPID, Integral_Limit, 200, 200);
    PidInitMode(&TD_t.Roll_Motor.PPID, Output_Limit, 5000, 0);
    TD_t.Roll_Motor.speedMin = -2000;
    TD_t.Roll_Motor.speedMax = 2000;

    //yaw motor speed pid init
    PidInit(&TD_t.Yaw_Motor.SPID, Spid[4][0], Spid[4][1], Spid[4][2], Integral_Limit | Output_Limit);
    PidInitMode(&TD_t.Yaw_Motor.SPID, Integral_Limit, 200, 200);
    PidInitMode(&TD_t.Yaw_Motor.SPID, Output_Limit, 10000, 0);

    //yaw motor position pid init
    PidInit(&TD_t.Yaw_Motor.PPID, Ppid[4][0], Ppid[4][1], Ppid[4][2], Integral_Limit | Output_Limit);
    PidInitMode(&TD_t.Yaw_Motor.PPID, Integral_Limit, 200, 200);
    PidInitMode(&TD_t.Yaw_Motor.PPID, Output_Limit, 10000, 0);


//
//    // Last Joint motor speed pid and position pid init
//    PidInit(&TD_t.Last_Joint[0].SPID, Spid[6][0], Spid[6][1], Spid[6][2], Integral_Limit | Output_Limit);
//    PidInitMode(&TD_t.Last_Joint[0].SPID, Integral_Limit, 200, 200);
//    PidInitMode(&TD_t.Last_Joint[0].SPID, Output_Limit, 10000, 0);
//
//    // Last Joint motor speed pid and position pid init
//    PidInit(&TD_t.Last_Joint[0].PPID, Ppid[6][0], Ppid[6][1], Ppid[6][2], Integral_Limit | Output_Limit);
//    PidInitMode(&TD_t.Last_Joint[0].PPID, Integral_Limit, 200, 200);
//    PidInitMode(&TD_t.Last_Joint[0].PPID, Output_Limit, 10000, 0);
//
//
//
//    // Last Joint motor speed pid and position pid init
//    PidInit(&TD_t.Last_Joint[1].SPID, Spid[6][0], Spid[6][1], Spid[6][2], Integral_Limit | Output_Limit);
//    PidInitMode(&TD_t.Last_Joint[1].SPID, Integral_Limit, 200, 200);
//    PidInitMode(&TD_t.Last_Joint[1].SPID, Output_Limit, 10000, 0);
//
//    // Last Joint motor speed pid and position pid init
//    PidInit(&TD_t.Last_Joint[1].PPID, Ppid[6][0], Ppid[6][1], Ppid[6][2], Integral_Limit | Output_Limit);
//    PidInitMode(&TD_t.Last_Joint[1].PPID, Integral_Limit, 200, 200);
//    PidInitMode(&TD_t.Last_Joint[1].PPID, Output_Limit, 10000, 0);


    //夹取模块初始化
    //    Grasp.C_t = &Clip_t;
    //    Clip_t.Clip_Motor[0].Encoder=Encoder_Init(M3508,7 );
    //    Clip_t.Clip_Motor[1].Encoder=Encoder_Init(M3508, 8);
    //    PID_Init(&Clip_t.Clip_Motor[0].SPID, 1, 0, 2, 4000, 10);
    ////    PID_Init(&Clip_t.Clip_Motor[1].SPID, 1, 0, 0, 2000, 10);
    //    PID_Init(&Clip_t.Clip_Motor[0].PPID, 1, 0, 2, 4000, 10);
    //    PID_Init(&Clip_t.Clip_Motor[1].SPID, 1, 0, 0, 2000, 10);

    //绑定CAN1接收回调函数 bind the CAN1 Rx callback function
    ECF_CAN_Rx_Callback_Register(&can1_manage, &MotorDataDeal);

//    PWM初始化
    ECF_PWM_50HZ_Output_Init(&htim1, TIM_CHANNEL_1);
    //set can standard id
    Gimbal_Can_msg.std_id = 0x200;
    Gimbal_Can_msg.dlc = 8;

}

void Gimbal_PowerOff_Drive(Grasp_t *G) {

}



int psc = 2000;
int dir=1;
int pwm = 550;
uint8_t Can_Data[8] = {0};
static int pos_lock;
static int pos[2];

void Forword_Motor_Drive(Three_D_Arm_t *TD,int32_t X_IN) {//电机为负时上升
    PidCalculate(&TD->Forward_Motor[0].SPID, X_IN, TD->Forward_Motor[0].Encoder->Speed[1]);
    CAN1_C620_OR_C610_201_TO_204_SendMsg(TD->Forward_Motor[0].SPID.out, 0, 0, 0);
}


void Grasp_Motor_Drive(void) {
}




static bool_t Sucker_Lock = 0;//
static int32_t LastJoint_LockPosition[2];
static int32_t Pitch_LockPosition;
static int32_t Roll_LockPosition;
static fp32 Yaw_LockPosition;
static int32_t LastJointGain=2;
static int speed=0;
/************************** Dongguan-University of Technology -ACE**************************
 * @brief
 *
 * @param is_Auto_Mode 判断是否为自动还是手动模式，手动模式具有完整的自由度与控制，自动模式只有几个档位可调
 * @param roll
 * @param pitch
 * @param yaw
 * @param sucker_state
 ************************** Dongguan-University of Technology -ACE***************************/
float k1 = -0.5f;
float k2 = 1.0f;
void Arms_Drive(Three_D_Arm_t *Arm_t, int16_t roll, int16_t pitch, int16_t yaw, int16_t joint , int16_t  forward ,bool_t update_sucker_state) {
    fp32 see_T_yaw,see_current_yaw;
//    if (update_sucker_state != Arm_t->Suker_state) {
//        if (update_sucker_state == 1) {
//            //输入上升沿时toggle lock
//            Arm_t->Suker_state = 1 - Arm_t->Suker_state;
//            // wirte pin
//        }
//    }

    //cal arccos for pitch
    float l1_square ;
    float l2_square ;
    float l3_square ;
    arm_power_f32(&Arm_t->l1, 1, &l1_square);
    arm_power_f32(&Arm_t->l2, 1, &l2_square);
    arm_power_f32(&Arm_t->l3, 1, &l3_square);

    Arm_t->Pitch1_Angle = acosf((l1_square + l3_square - l2_square) / (2 * Arm_t->l1 * Arm_t->l2)) + Arm_t->l3ToHorizontalPlane_Angle;
    Arm_t->Pitch2_Angle = acosf((l1_square + l2_square - l3_square) / (2 * Arm_t->l1 * Arm_t->l2));
    //TODO:output


	PidCalculate(&Arm_t->Forward_Motor[0].SPID, forward*(-10), Arm_t->Forward_Motor[0].Encoder->Speed[1]);

    //控制横移部分
//    PidCalculate(&Arm_t->Traverse_Motor.SPID, traverse * 200, Arm_t->Traverse_Motor.Encoder->Speed[1]);

    if (pitch!=0) Pitch_LockPosition = Arm_t->Pitch_Motor.Encoder->Encode_Record_Val + pitch;

    motor_position_speed_control(&Arm_t->Pitch_Motor.SPID,
                                 &Arm_t->Pitch_Motor.PPID,
                                 Pitch_LockPosition,
                                 Arm_t->Pitch_Motor.Encoder->Encode_Record_Val,
                                 Arm_t->Pitch_Motor.Encoder->Speed[1]);

    if(roll!=0) Roll_LockPosition = Arm_t->Roll_Motor.Encoder->Encode_Record_Val + roll;

    motor_position_speed_control(&Arm_t->Roll_Motor.SPID,
                                 &Arm_t->Roll_Motor.PPID,
                                 Roll_LockPosition,
                                 Arm_t->Roll_Motor.Encoder->Encode_Record_Val,
                                 Arm_t->Roll_Motor.Encoder->Speed[1]);

    if(yaw!=0) Yaw_LockPosition  += yaw / 660.0f * 8192.0f / 360.0f * 4.0f;

//    motor_position_speed_control(&Arm_t->Yaw_Motor.SPID,
//                                 &Arm_t->Yaw_Motor.PPID,
//                                 Yaw_LockPosition,
//                                 Arm_t->Yaw_Motor.Encoder->Encode_Record_Val,
//                                 Arm_t->Yaw_Motor.Encoder->Speed[1]);

	see_T_yaw = (Arm_t->Yaw_Motor.Encoder->Encode_Record_Val - Yaw_LockPosition) / 57.295779513f * k1 - (Arm_t->Yaw_Motor.Encoder->Speed[1]*3.14159f/30.0f) * k2  ;
   see_current_yaw = see_T_yaw / 0.741f * 10000;
   Arm_t->Yaw_Motor.SPID.out = (see_current_yaw - 128.3507f) / 0.7778f;
   Arm_t->Yaw_Motor.SPID.out = abs_limit(Arm_t->Yaw_Motor.SPID.out,29000);


//    speed=Arm_t->Roll_Motor.Encoder->Speed[1];
//    MotorVelocityCurve(&Arm_t->Roll_Motor);
    //    CAN1_C620_OR_C610_201_TO_204_SendMsg(Arm_t->Forward_Motor[0].SPID.out, 0, 0, 0);
    //    CAN1_C620_OR_C610_205_TO_208_SendMsg(Arm_t->Pitch_Motor.SPID.out, Arm_t->Roll_Motor.SPID.out, 0, Arm_t->Traverse_Motor.SPID.out);
    CAN1_C620_OR_C610_205_TO_208_SendMsg(Arm_t->Pitch_Motor.SPID.out, Arm_t->Roll_Motor.SPID.out, 0,Arm_t->Traverse_Motor.SPID.out);
    CAN1_C620_OR_C610_201_TO_204_SendMsg(Arm_t->Forward_Motor[0].SPID.out, 0, 0, 0);
    CAN1_GM6020_5_TO_8_SendMsg(Arm_t->Yaw_Motor.SPID.out, 0, 0, 0);


//    pwm+=joint/60;
//    if (pwm>2300)pwm=2300;
//    else if(pwm<500)pwm=500;
//    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
}
void Three_Degrees_Arms_Init() {
    Pitch_LockPosition=TD_t.Pitch_Motor.Encoder->Encode_Record_Val;
    Roll_LockPosition=TD_t.Roll_Motor.Encoder->Encode_Record_Val;
    Yaw_LockPosition=TD_t.Yaw_Motor.Encoder->Encode_Record_Val;
    pwm=1000;
}

//函数指针，根据枚举来选
//void (*pCalCurve[])(CurveObjectType *curve)={CalCurveNone,CalCurveTRAP,CalCurveSPTA};
/*S型曲线速度计算*/
static void CalCurveSPTA(Motor_t *spta)
{
    float power=0.0;
    float speed=0.0;
    //2t-Tmax/Tmax
    power=(2*((float)spta->aTimes)-((float)spta->maxTimes))/((float)spta->maxTimes);
    //-flex(2t-Tmax/Tmax)
    power=(0.0-spta->flexible)*power;
    //1+e^(-flex(2t-Tmax/Tmax))
    speed=1+expf(power);
    //(Vtarget-Vstart)/(1+e^(-flex(2t-Tmax/Tmax)))
    speed=(spta->targetSpeed-spta->startSpeed)/speed;
    spta->currentSpeed=speed+spta->startSpeed;

    if(spta->currentSpeed>spta->speedMax)
    {
        spta->currentSpeed=spta->speedMax;
    }

    if(spta->currentSpeed<spta->speedMin)
    {
        spta->currentSpeed=spta->speedMin;
    }

}
/* 电机曲线加减速操作-------------------------------------------------------- */
void MotorVelocityCurve(Motor_t *curve)
{
    curve->currentSpeed = (float )curve->Encoder->Speed[1];
    float temp=0;
    //如果速度>折返
    if(curve->targetSpeed>curve->speedMax)
    {
        curve->targetSpeed=curve->speedMax;
    }

    if(curve->targetSpeed<curve->speedMin)
    {
        curve->targetSpeed=curve->speedMin;
    }
    //当前和初始速度绝对值小于加速度
    if((fabs(curve->currentSpeed-curve->startSpeed)<=curve->stepSpeed)&&(curve->maxTimes==0))    {
        if(curve->startSpeed<curve->speedMin)
        {
            curve->startSpeed=curve->speedMin;
        }

        temp=fabs(curve->targetSpeed-curve->startSpeed);
        temp=temp/curve->stepSpeed;
        curve->maxTimes=(uint32_t)(temp)+1;
        curve->aTimes=0;
    }

    if(curve->aTimes<curve->maxTimes)
    {
        CalCurveSPTA(curve);
        curve->aTimes++;
    }
    else
    {
        curve->currentSpeed=curve->targetSpeed;
        curve->maxTimes=0;
        curve->aTimes=0;
    }
    PidCalculate(&curve->SPID, curve->currentSpeed, curve->Encoder->Speed[1]);
}

