#ifndef __BOARDCOMMUNI_H_
#define __BOARDCOMMUNI_H_
#include "stm32f4xx.h"
#include "RemoteDeal.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "usart.h"


//板间通讯，不同指令的ID
#define CAN2_A2B_RC_CH_NUM 0x5ff
#define CAN2_A2B_RC_SW_NUM 0x6ff




/*****************函数定义*****************/
void USART_A2B_RC_Send(void);
void USART_A2B_KM_Send(void);
void CAN_A2B_RC_Send(void);
void CAN_A2B_KM_Send(void);

#endif
