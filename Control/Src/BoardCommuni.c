/************************** Dongguan-University of Technology -ACE**************************
 * @file BoardCommuni.c
 * @brief
 * @author pansyhou侯文辉 (1677195845lyb@gmail.com)
 * @version 1.0
 * @date 2022-07-12
 *
 *
 * @history
 * <table>
 * Date       Version Author Description
 * 2022-07-12   1.0   侯文辉
 * @verbatim
 * ==============================================================================
 *  本来写了很多都在底盘板接收遥控的，但是21还是20年出现过底盘电池或者其他干扰较大，上场动不了，
 *  于是遥控接收就调整到云台板收，将部分数据用串口+DMA发送，减少CPU占用提高发送效率
 *
 * 同时写一套用CAN通讯的版本，冗余备用
 * ==============================================================================
 * @endverbatim
 ************************** Dongguan-University of Technology -ACE***************************/
#include "BoardCommuni.h"
#include "main.h"
#include "Gimbal_Task.h"
#include "RemoteDeal.h"
#include "SYSInit.h"
#include "stdlib.h"
#include "bsp_can.h"
#include "bsp_dr16.h"
#include "fifo.h"
#include "stm32f4xx_hal_dma.h"
extern Gimbal_t Gimbal;
//底盘测试模式
#define Chassis_TEST_MODE 0

#define Data_Size 12
uint8_t RC_buffer[Data_Size];
extern REMOTE_t REMOTE;

#define RC_CHANNAL_ERROR_VALUE 700

#if Chassis_TEST_MODE
uint8_t Jscope_RC_s1;
uint8_t Jscope_RC_s2;
int16_t Jscope_RC_ch0;
int16_t Jscope_RC_ch1;
int16_t Jscope_RC_ch2;

#endif


/*****************普普通通串口版本*********************/

///************************** Dongguan-University of Technology -ACE**************************
// * @brief 云台A板遥控RC数据发送到底盘B板
// * （除了键盘
// ************************** Dongguan-University of Technology -ACE***************************/
//void USART_A2B_RC_Send(void)
//{

//  RC_buffer[0] = REMOTE.RC_ctrl->rc.ch[0];
//  RC_buffer[1] = (REMOTE.RC_ctrl->rc.ch[0] >> 8);

//  RC_buffer[2] = REMOTE.RC_ctrl->rc.ch[1];
//  RC_buffer[3] = (REMOTE.RC_ctrl->rc.ch[1] >> 8);

//  RC_buffer[4] = REMOTE.RC_ctrl->rc.ch[2];
//  RC_buffer[5] = (REMOTE.RC_ctrl->rc.ch[2] >> 8);

//  RC_buffer[6] = REMOTE.RC_ctrl->rc.ch[3];
//  RC_buffer[7] = (REMOTE.RC_ctrl->rc.ch[3] >> 8);

//  RC_buffer[8] = REMOTE.RC_ctrl->rc.ch[4];
//  RC_buffer[9] = (REMOTE.RC_ctrl->rc.ch[4] >> 8);

//  RC_buffer[10] = REMOTE.RC_ctrl->rc.s1;
//  RC_buffer[11] = REMOTE.RC_ctrl->rc.s2;

//#if Chassis_TEST_MODE //测试模式，看串口丢包情况
//  Jscope_RC_s1 = REMOTE.RC_ctrl->rc.s1;
//  Jscope_RC_s2 = REMOTE.RC_ctrl->rc.s2;
//  Jscope_RC_ch0 = REMOTE.RC_ctrl->rc.ch[0];
//  Jscope_RC_ch1 = REMOTE.RC_ctrl->rc.ch[1];
//  Jscope_RC_ch2 = REMOTE.RC_ctrl->rc.ch[2];
//#endif

//  HAL_UART_Transmit(&huart1, (uint8_t *)RC_buffer, Data_Size, 50);
//}

///************************** Dongguan-University of Technology -ACE**************************
// * @brief 云台A板遥控keyboard & Mouse数据发送到底盘B板
// *
// ************************** Dongguan-University of Technology -ACE***************************/
//void USART_A2B_KM_Send(void)
//{
//  RC_buffer[0] = REMOTE.RC_ctrl->mouse.x;
//  RC_buffer[1] = (REMOTE.RC_ctrl->mouse.x >> 8);

//  RC_buffer[2] = REMOTE.RC_ctrl->mouse.y;
//  RC_buffer[3] = (REMOTE.RC_ctrl->mouse.y >> 8);

//  RC_buffer[4] = REMOTE.RC_ctrl->mouse.press_l;
//  RC_buffer[5] = REMOTE.RC_ctrl->mouse.press_r;

//  RC_buffer[6] = REMOTE.RC_ctrl->key.v;
//  RC_buffer[7] = (REMOTE.RC_ctrl->key.v >> 8);

//  RC_buffer[8] = 0;
//  RC_buffer[9] = 0;

//  RC_buffer[10] = REMOTE.RC_ctrl->rc.s1;
//  RC_buffer[11] = REMOTE.RC_ctrl->rc.s2;

//  HAL_UART_Transmit(&huart1, (uint8_t *)RC_buffer, Data_Size, 50);
//}




/****************CAN版本*****************/
static uint8_t RC_Data1[8];
static uint8_t RC_Data2[8];
void CAN_A2B_RC_Send(void)
{
    RC_Data1[0] = Gimbal.RC->RC_ctrl->rc.ch[0] >> 8;
    RC_Data1[1] = (uint8_t)Gimbal.RC->RC_ctrl->rc.ch[0] ;
    RC_Data1[2] = Gimbal.RC->RC_ctrl->rc.ch[1] >> 8;
    RC_Data1[3] = (uint8_t)Gimbal.RC->RC_ctrl->rc.ch[1] ;
    RC_Data1[4] = Gimbal.RC->RC_ctrl->rc.ch[2] >> 8;
    RC_Data1[5] = (uint8_t)Gimbal.RC->RC_ctrl->rc.ch[2] ;
    RC_Data1[6] = Gimbal.RC->RC_ctrl->rc.ch[3] >> 8;
    RC_Data1[7] = (uint8_t)Gimbal.RC->RC_ctrl->rc.ch[3] ;

    ECF_CAN_Send_Msg_FIFO(&hcan2,0x100, RC_Data1, 8);
	
    vTaskDelay(2);
	
    RC_Data2[0] = Gimbal.RC->RC_ctrl->rc.ch[4] >> 8;
    RC_Data2[1] = (uint8_t)Gimbal.RC->RC_ctrl->rc.ch[4] ;
    RC_Data2[2] = Gimbal.RC->RC_ctrl->rc.s1;
    RC_Data2[3] = Gimbal.RC->RC_ctrl->rc.s2;
    RC_Data2[4]=0;
    RC_Data2[5]=0;
    RC_Data2[6]=0;
    RC_Data2[7]=0;

    ECF_CAN_Send_Msg_FIFO(&hcan2,0x102, RC_Data2, 8);
}

void CAN_A2B_KM_Send(void)
{

  // CAN2_SendMsg_1(CAN2_A2B_RC_CH_NUM,REMOTE.RC_ctrl->rc.ch[0],REMOTE.RC_ctrl->rc.ch[1],REMOTE.RC_ctrl->rc.ch[2],REMOTE.RC_ctrl->rc.ch[3]);

  // CAN2_SendMsg_2(CAN2_A2B_RC_SW_NUM,REMOTE.RC_ctrl->rc.s1,REMOTE.RC_ctrl->rc.s2,0,0);
}





/*******************串口+环形队列版本*****************/

//#define USART1_RX_LEN 256
//#define USART1_TX_LEN 512
//static uint8_t Usart1_Rx[USART1_RX_LEN] = {0};
//static uint8_t Usart1_Tx[USART1_TX_LEN] = {0};
//static uint8_t Usart1_Rx_Buffer[USART1_RX_LEN] = {0};
//static uint8_t Usart1_Tx_Buffer[USART1_TX_LEN] = {0};

//fifo_rx_def fifo_usart1_rx;
//fifo_rx_def fifo_usart1_tx;

//void BC_UART1_TX_Init(void)
//{
//  /* 使能串口中断 */
//  /* 使能串口中断 */
//  //	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);  // 接收数据寄存器不为空中断
//  //	__HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);  // 传输数据寄存器空中断
//  __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);   // 开启传输完成中断
//  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); // 使能空闲中断

//  /* 使能串口DMA发送 */
//  __HAL_DMA_DISABLE(huart1.hdmatx); // 关闭串口DMA发送通道 （不用开启循环模式）
//  if (fifo_init(&fifo_usart1_tx, Usart1_Tx_Buffer, USART1_TX_LEN) == -1)
//  {
//    Error_Handler(); // 必须 2 的幂次方
//  }
//}

//void BC_UART1_RX_Init(void)
//{
//  /* 使能串口中断 */
//  /* 使能串口中断 */
//  //	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);  // 接收数据寄存器不为空中断
//  //	__HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);  // 传输数据寄存器空中断
//  // __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);    // 开启传输完成中断
//  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); // 使能空闲中断

//  /* 使能串口DMA接收 */
//  UART_Start_Receive_DMA(&huart1, Usart1_Rx, USART1_RX_LEN); // 开启串口DMA接收（记得开启循环模式）
//  if (fifo_init(&fifo_usart1_rx, Usart1_Rx_Buffer, USART1_RX_LEN) == -1)
//  {
//    Error_Handler(); // 必须 2 的幂次方
//  }
//}

////发送
//uint32_t usart1_dma_send(uint8_t *data, uint16_t len)
//{
//  uint32_t result = fifo_write_buff(&fifo_usart1_tx, data, len); //将数据放入循环缓冲区,返回的是数据长度

//  if (result != 0 && huart1.gState == HAL_UART_STATE_READY) //当没有接收的时候l,防止数据被冲掉
//  {
//    len = fifo_read_buff(&fifo_usart1_tx, Usart1_Tx, USART1_TX_LEN); //从循环缓冲区获取数据

//    if (HAL_UART_Transmit_DMA(&huart1, Usart1_Tx, len) != HAL_OK) //开启dma传输
//    {
//      // Error_Handler();
//    }
//  }

//  if (result == len)
//  {
//    return len;
//  }
//  else
//  {
//    return result;
//  }
//}

////接收,这个是不清掉DMA_FLAG_GL6全局标志位的版本，放在hal自己定义的下面先试试行不行
//void USART1_IRQHandler_callback1(void)
//{
//  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) == SET) //空闲中断
//  {
//    volatile uint32_t num = 0;

//    num = huart1.Instance->SR; //清除RXNE标志
//    num = huart1.Instance->DR; //清USART_IT_IDLE标志

//    __HAL_DMA_DISABLE(huart1.hdmarx); // 关闭串口DMA发送通道

//    //! 获取DMA中未传输的数据个数，NDTR寄存器分析参考中文参考手册 （DMA_Channel_TypeDef）  这个不同的芯片HAL库里面定义的命名有点不同
//    num = USART1_RX_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

//    fifo_write_buff(&fifo_usart1_rx, Usart1_Rx, num);

//    
//    // HAL_UART_Receive_DMA(&huart1, Usart1_Rx, USART1_RX_LEN); // 启动DMA接收
//  }
//}


////第二段，重启DMA接收
//void USART1_IRQHandler_callback2(void)
//{
//  HAL_UART_Receive_DMA(&huart1, Usart1_Rx, USART1_RX_LEN); // 启动DMA接收
//}




