/*
 * @Author: your name
 * @Date: 2021-01-09 15:32:34
 * @LastEditTime: 2021-01-12 14:35:44
 * @LastEditors: Please set LastEditors
 * @Description: 定义PC键盘,遥控结构体,键盘结构体
 */
#ifndef __REMOTEDEAL_H_
#define __REMOTEDEAL_H_
#include "bsp_dr16.h"
#include "filter.h"
#include <Detect_Task.h>

/* ----------------------- PC Key Definition-------------------------------- */

typedef enum {
    Follow_Independent = 0,
    Arm_Independent = 1,
    Global_Status_End
} Global_Status_e;


/****************键盘控制状态结构体**************************/
typedef __packed struct
{
//	unsigned char Electromagnet : 1; //电磁铁
//	unsigned char Magazine : 1;		 //弹仓
//	unsigned char Clip : 1;			 //A板
//	unsigned char Auto_Clamp : 1;	 //自动夹取
//	unsigned char Up_motor : 1;		 //抬升电机
//	unsigned char Flex_motor : 1;	 //伸缩电机
//	unsigned char Flip_motor : 1;	 //翻转电机
//	unsigned char Clamp_motor : 1;	 //夹子电机
//
//	/* -----------------------------以下为新结构体------------------------ */
//
//	/* ----------------------------底盘状态 -------------------------------*/
//	unsigned char Rotation : 2;	   //大陀螺
//	unsigned char Wiggle : 2;	   //扭腰
//	unsigned char Independent : 2; //独立行走
//
//	/* ----------------------------上装状态------------------------------- */
//	unsigned char Grasp_Up : 2;	  //自动夹取
//	unsigned char Translation  : 1; //上装复位
//	unsigned char Telescoping : 1;	  //松夹
//	unsigned char Clap : 1;	  //兑换一个
//	unsigned char Flip : 1;	  //兑换两个
//
//	/*-----------------------------底盘功能状态----------------------------*/
//	unsigned char RFID : 1;	   //救援卡
//	unsigned char Barrier : 1; //救援爪
//	unsigned char Gimbal_Yaw :1;//云台yaw轴

    //分为整车状态和自动状态
    // global status
    unsigned char Global_Status : 1 ; //0时为底盘独立模式，1为取矿独立模式

    //


} KeyBoard_State_t;

/*Remote结构体*/
typedef __packed struct
{
    RC_ctrl_t *RC_ctrl;
	first_order_filter_type_t RC_X;
    first_order_filter_type_t RC_Y;
    first_order_filter_type_t RC_Z;
    first_order_filter_type_t KM_X;
    first_order_filter_type_t KM_Y;
    first_order_filter_type_t KM_Z;
	KeyBoard_State_t state;
} REMOTE_t;

/*Remote结构体初始化*/
void Remote_Data_Init(void);

/*Remote数据处理*/
void Remote_Data_Deal(void);

/*返回Remote数据指针*/
REMOTE_t *Return_RemoteDeal_Point(void);

#endif
