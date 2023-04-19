#ifndef __Distance_Sensor_H
#define __Distance_Sensor_H
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "usart.h"


extern DMA_HandleTypeDef hdma_usart6_rx;

typedef __packed struct
{
	uint8_t data[64];
	float InitDistance;
	float distance; 
	uint8_t receive_flag;
	float Lock_Distance;
}VL53L0_t;

void Sensor_Data_Deal(void);
VL53L0_t *Return_VL53L0_t_Pointer(void);
void Sensor_RX_Init(void);
void DS_DataProcess(uint8_t *Data);
void DSensor_UART_IRQHandler(UART_HandleTypeDef *huart);
#endif

