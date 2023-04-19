#include "Gimbal_Task.h"
#include "bsp_dr16.h"
#include "SYSInit.h"
#include "Gimbal_Fsm.h"
#include "bsp_usb.h"

Gimbal_t Gimbal;
//TODO:��ʼ������


static void Gimbal_Init(void)
{
    //����ӳ��
	Gimbal.RC=Return_RemoteDeal_Point();     //��ȡң�����������ݺ�Ľṹ��ָ��
	Gimbal.Gimbal_Fsm=Return_Gimbal_FSM();
	Gimbal.Graps=Return_Grasp_t_Pointer();
	//����FSM��ʼ��
	Gimbal_Fsm_Init();
    Gimbal_Motor_Init();
	
    //״̬����ӳ��(��ChassisMotor��)
	Gimbal.PowerOff=Gimbal_PowerOff_Drive;
    //���ݳ�ʼ��
//    Gimbal.RC=Return_RemoteDeal_Point();     //��ȡң����ָ��
//    Chassis.Wheel_Init(&Chassis.C);     //���ӳ�ʼ��
//    Chassis.Fsm=Return_Chassis_FSM();   //��ȡfsm״̬��ָ��
//    Chassis.Fsm_Init();                 //״̬����ʼ��
}

volatile int32_t  left=0,right=0;
/***
 *                    _ooOoo_
 *                   o8888888o
 *                   88" . "88
 *                   (| -_- |)
 *                    O\ = /O
 *                ____/`---'\____
 *              .   ' \\| |// `.
 *               / \\||| : |||// \
 *             / _||||| -:- |||||- \
 *               | | \\\ - /// | |
 *             | \_| ''\---/'' | |
 *              \ .-\__ `-` ___/-. /
 *           ___`. .' /--.--\ `. . __
 *        ."" '< `.___\_<|>_/___.' >'"".
 *       | | : `- \`.;`\ _ /`;.`/ - ` : | |
 *         \ \ `-. \_ __\ /__ _/ .-` / /
 * ======`-.____`-.___\_____/___.-`____.-'======
 *                    `=---='
 *
 * .............................................
 *          ���汣��             ����BUG
 */
void Gimbal_Task(void *pvParameters)
{
	Gimbal_Init();
	vTaskDelay(500);
	while(1)
	{
		FSM_Deal(Gimbal.Gimbal_Fsm,Gimbal.RC->RC_ctrl->rc.s1,Gimbal.RC->RC_ctrl->rc.s2);
		vTaskDelay(10);
	}
	
}

