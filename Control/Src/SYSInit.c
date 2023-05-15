#include "SYSInit.h"
//#include "usb_device.h"
#include "bsp_pwm.h"
//#include "Distance_Sensor.h"

//#include "SEGGER_SYSVIEW.h"
extern TIM_HandleTypeDef htim4;
extern SPI_HandleTypeDef hspi1;

#define DEBUG_Mode 1

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 系统硬件配置初始化，必须写在FreeRTOS开始之前
 *
 ************************** Dongguan-University of Technology -ACE***************************/
void Sys_Init(void)
{
    HAL_GPIO_WritePin(GPIOH, LED_Red_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, LED_Green_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, LED_Blue_Pin, GPIO_PIN_RESET);
#if DEBUG_Mode
//	SEGGER_SYSVIEW_Conf(); // SystemView调试初始化
//	// SEGGER_SYSVIEW_Start();//
//
//	//初始化DWT计数器,SystemView会从DWT的Cycle counter获取系统当前周期数用于生成时间戳
//	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//	DWT->CYCCNT = 0;
//	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
//	DWT_Init(168);
//  while (BMI088_init(&hspi1, 1) != BMI088_NO_ERROR);
    ECF_RC_Init();
    ECF_CAN_Init();
	//遥控初始化
	Remote_Data_Init();

    //蜂鸣器
//	HAL_TIM_Base_Start(&htim4);
//	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
//
//	Sensor_RX_Init();
//	vTaskDelay(100);
}
