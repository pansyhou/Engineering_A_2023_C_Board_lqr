#include "BoardCommuni_Task.h"
#include "BoardCommuni.h"
#include "SYSInit.h"




void BoardCommuni_Task(void *pvParameters)
{

	while (1)
	{
		CAN_A2B_RC_Send();
		vTaskDelay(10);
	}
}
