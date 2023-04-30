#ifndef __SYS_INIT_H
#define __SYS_INIT_H

//global variable
#define ARM_L1_LENTH 23.0f
#define ARM_L2_LENTH 23.0f
#define INIT_ARM_L3_LENTH 23.0f

#define PWM_CMP_UPPER_LIMIT 2500
#define PWM_CMP_LOWER_LIMIT 500

/*  系统头文件 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stdint.h"

/************************* Task ************************/
// #include "Task_Chassis.h"
// #include "Task_Gimbal.h"
// #include "Task_Fire.h"
// #include "Chassis_Task.h"
/* ************************freertos******************** */
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
	
/* ************************ Hardward ******************** */	
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"



/* ************************ CONTROL ******************** */	

#include "RemoteDeal.h"

/* ************************ ALGORITHM ******************** */	
#include "tim.h"
 #include "pid.h"
#include "rmmotor.h"
 #include "maths.h"
/************************* BSP *************************/
#include "bsp_can.h"
#include "bsp_dr16.h"
#include "bsp_dwt.h"
#include "BMI088driver.h"


void Sys_Init(void);


#endif // !1
