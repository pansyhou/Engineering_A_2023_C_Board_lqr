#include "bsp_Distance_Sensor.h"
#include "usart.h"
#include "stdlib.h"
#include "string.h"
//#include "Referee_Drive.h"
//#include "SEGGER_SYSVIEW.h"
#define DataSize 64
#define DataBuffer 128
uint8_t Sensor_RxData[DataSize];
uint8_t Sensor_RxData2[DataSize];
VL53L0_t VL53L0;
VL53L0_t *Return_VL53L0_t_Pointer(void)
{
	return &VL53L0;
}
void Sensor_RX_Init(void)
{
//	HAL_UART_Receive_IT(&huart6, Sensor_RxData, DataSize);	//中断接收，狗都不用
//	HAL_UART_Receive_DMA(&huart6, Sensor_RxData, DataSize);//这个是单缓冲+循环接收的，具有数据被冲掉的风险
    //引用裁判系统的初始化，毕竟懒得写了，我已经提高裁判系统的初始化适用性了
//    usart_DoubleBuffer_init(&huart6,Sensor_RxData,Sensor_RxData2,DataSize);

}

//记得加在串口DMA中断
//uint16_t this_time_rx = 0;
//void DSensor_UART_IRQHandler(UART_HandleTypeDef *huart)
//{
////	SEGGER_SYSVIEW_RecordEnterISR();
////TODO:记得测试代码
//    static volatile uint8_t res;
//    if (huart->Instance->SR & UART_FLAG_IDLE)//这里的USART6->SR改成了huart.Instance->SR，提高适用性
//    {
//        __HAL_UART_CLEAR_PEFLAG(huart);
//
//        if ((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            __HAL_DMA_DISABLE(huart->hdmarx);
//            this_time_rx = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart->hdmarx);
//            __HAL_DMA_SET_COUNTER(huart->hdmarx, USART_RX_BUF_LENGHT);
//            huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
//            __HAL_DMA_ENABLE(huart->hdmarx);
//            DS_DataProcess(Sensor_RxData);
//
//            // detect_hook(REFEREE_TOE);
//        }
//        else
//        {
//            __HAL_DMA_DISABLE(huart->hdmarx);
//            this_time_rx = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart->hdmarx);
//            __HAL_DMA_SET_COUNTER(huart->hdmarx, USART_RX_BUF_LENGHT);
//            huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
//            __HAL_DMA_ENABLE(huart->hdmarx);
//            DS_DataProcess(Sensor_RxData2);
//            // detect_hook(REFEREE_TOE);
//        }
//    }
//	DS_DataProcess(Sensor_RxData);
////	SEGGER_SYSVIEW_RecordExitISR();
//}

void DS_DataProcess(uint8_t *Data)
{
	char *str;
	// VL53L0返回的是一个字符串d:  ,从中截取数字子字符串
	str = strstr((char *)Data, (char *)"d:  ");
	str += 3;
	float Distance = atof(str); //字符串转浮点数
	if (Distance > 30)
	{
		VL53L0.distance = Distance;
	}
}
