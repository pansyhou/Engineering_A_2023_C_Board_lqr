/***
*                    .::::.
*                  .::::::::.
*                 :::::::::::  FUCK YOU
*             ..:::::::::::'
*           '::::::::::::'
*             .::::::::::
*        '::::::::::::::..
*             ..::::::::::::.
*           ``::::::::::::::::
*            ::::``:::::::::'        .:::.
*           ::::'   ':::::'       .::::::::.
*         .::::'      ::::     .:::::::'::::.
*        .:::'       :::::  .:::::::::' ':::::.
*       .::'        :::::.:::::::::'      ':::::.
*      .::'         ::::::::::::::'         ``::::.
*  ...:::           ::::::::::::'              ``::.
* ```` ':.          ':::::::::'                  ::::..
*                    '.:::::'                    ':'````..
*/

#include "Gimbal_Fsm.h"
#include "Gimbal_Task.h"


extern Gimbal_t Gimbal;
FSM_t Gimbal_Fsm;


State_t OFFLINE;    //离线模式
State_t LIFT;       //供测试用模式
State_t Arm_Test;
State_t Mining;     //取矿模式

// State_t INDEPEN;    //独立模式
// State_t ROTATION;   //大陀螺模式
State_t Keyboard;   //键盘模式
State_t Gimbal_State_Table[State_Line][State_Column]; //状态参数表

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 底盘状态机控制指针获取
 * 
 * @return FSM_t* 
************************** Dongguan-University of Technology -ACE***************************/
FSM_t *Return_Gimbal_FSM(void)
{
    return &Gimbal_Fsm;
}

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 底盘状态机初始化
 * 
************************** Dongguan-University of Technology -ACE***************************/
void Gimbal_Fsm_Init(void)
{
    //TODO:这样的赋值是第一次试，记得测试
//    不知道为什么嘎了....，可能涉及到GNU方言，keil的工具链识别不出来
//     FSM_t Gimbal_Fsm = {
//             .Current_State=NULL,
//             .Last_State=NULL,
//             .State_Table=Gimbal_State_Table,
//             .State_Change=StateChange       //状态机状态变更函数
//     };
   Gimbal_Fsm.State_Table = Gimbal_State_Table;

   Gimbal_Fsm.Last_State = NULL;
   Gimbal_Fsm.Current_State = NULL;
   Gimbal_Fsm.State_Change = StateChange; //状态机状态变更函数

    /*OFFLINE状态初始化*/
   OFFLINE.State_Prepare = Offline_Prepare;
   OFFLINE.State_Process = Offline_State;
   OFFLINE.Behavior_Process = PowerOff_bhv;

   /*INDEPEN状态初始化*/
   LIFT.Behavior_Process = Lift_State;
   LIFT.State_Process = Lift_State;
   LIFT.State_Prepare = Lift_Prepare;

   Arm_Test.Behavior_Process = Arm_State;
   Arm_Test.State_Prepare = Arm_Prepare;
   Arm_Test.State_Process = Arm_State;

//    Mining.Behavior_Process = Mining_State;
//    Mining.State_Prepare = Mining_State;
//    Mining.State_Process = Mining_Prepare;

    Keyboard.State_Prepare = KeyBoard_Prepare;
    Keyboard.State_Process = KeyBoard_State;
    Keyboard.Behavior_Process = KeyBoard_bhv;

    /*底盘状态机初始化*/
    //132
    Gimbal_State_Table[0][0] = Keyboard;    //s1=1 ,s2=1
    Gimbal_State_Table[0][2] = LIFT;     //s1=1  s2=3
    Gimbal_State_Table[0][1] = LIFT;    //s1=1  s2=2

    Gimbal_State_Table[1][0] = LIFT;     //s1=2  s2=1
    Gimbal_State_Table[1][1] = OFFLINE;    //s1=2  s2=2
    Gimbal_State_Table[1][2] = LIFT;    //s1=2  s2=3

    Gimbal_State_Table[2][0] = LIFT;   //s1=3 s2=1
    Gimbal_State_Table[2][1] = LIFT; //s1=3 s2=2
    Gimbal_State_Table[2][2] = Arm_Test; //s1=3 s2=3


}





/***********************OFFLINE*************************/
/*OFFLINE状态准备函数*/
static void Offline_Prepare(void)
{

}

/*离线状态处理*/
static void Offline_State(void)
{
    Gimbal_Fsm.Current_State->Behavior_Process=PowerOff_bhv;
    
}

/*断电行为函数*/
static void PowerOff_bhv(void)
{
    Three_Degrees_Arms_Init();
    Gimbal.PowerOff(Gimbal.Graps);
}

/************************** Dongguan-University of Technology -ACE**************************
 * @brief 这个东西很迷啊，也暂时想不到什么好的方法卡死，夹矿模式，
 * 
************************** Dongguan-University of Technology -ACE***************************/
static void Lift_Prepare(void)
{
    HAL_GPIO_WritePin(GPIOH, LED_Red_Pin|LED_Green_Pin|LED_Blue_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_SET);

}

static void Lift_State(void)
{
    Gimbal_Fsm.Current_State->Behavior_Process=Lift_bhv;
}

static void Lift_bhv(void)
{
    Arms_Drive(Gimbal.Graps->TD_t, 0, 0, 0, 0, 0,1);
}




static void Arm_Prepare(void)
{
    HAL_GPIO_WritePin(GPIOH, LED_Red_Pin|LED_Green_Pin|LED_Blue_Pin , GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_Green_GPIO_Port,LED_Red_Pin,GPIO_PIN_SET);
}

static void Arm_State(void)
{
    Gimbal_Fsm.Current_State->Behavior_Process=Arm_bhv;
}

static void Arm_bhv(void)
{
    Arms_Drive(Gimbal.Graps->TD_t,  Gimbal.RC->RC_ctrl->rc.ch[0], Gimbal.RC->RC_ctrl->rc.ch[4]*3, -Gimbal.RC->RC_ctrl->rc.ch[2], Gimbal.RC->RC_ctrl->rc.ch[1],Gimbal.RC->RC_ctrl->rc.ch[3],1);
}



static void Mining_Prepare(void)
{
    HAL_GPIO_WritePin(GPIOH, LED_Red_Pin , GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, LED_Green_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, LED_Blue_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_Blue_GPIO_Port,LED_Blue_Pin,GPIO_PIN_SET);
}

static void Mining_State(void)
{
    Gimbal_Fsm.Current_State->Behavior_Process=Mining_bhv;
}

static void Mining_bhv(void)
{

}


static void KeyBoard_Prepare(void){
    HAL_GPIO_WritePin(GPIOH, LED_Red_Pin|LED_Green_Pin|LED_Blue_Pin , GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_Green_GPIO_Port,LED_Blue_Pin,GPIO_PIN_SET);
}
static void KeyBoard_State(void){

}

//#define KEY_PRESSED_OFFSET_W       ((uint16_t)1 << 0)     //控制底盘
//#define KEY_PRESSED_OFFSET_S       ((uint16_t)1 << 1)
//#define KEY_PRESSED_OFFSET_A       ((uint16_t)1 << 2)
//#define KEY_PRESSED_OFFSET_D       ((uint16_t)1 << 3)
//#define KEY_PRESSED_OFFSET_SHIFT   ((uint16_t)1 << 4)     //加速用
//#define KEY_PRESSED_OFFSET_CTRL    ((uint16_t)1 << 5)     //减速用
//#define KEY_PRESSED_OFFSET_Q       ((uint16_t)1 << 6)     //切换模式
//#define KEY_PRESSED_OFFSET_E       ((uint16_t)1 << 7)
//#define KEY_PRESSED_OFFSET_R       ((uint16_t)1 << 8)
//#define KEY_PRESSED_OFFSET_F       ((uint16_t)1 << 9)
//#define KEY_PRESSED_OFFSET_G       ((uint16_t)1 << 10)    //救援切换模式
//#define KEY_PRESSED_OFFSET_Z       ((uint16_t)1 << 11)
//#define KEY_PRESSED_OFFSET_X       ((uint16_t)1 << 12)
//#define KEY_PRESSED_OFFSET_C       ((uint16_t)1 << 13)
//#define KEY_PRESSED_OFFSET_V       ((uint16_t)1 << 14)
//#define KEY_PRESSED_OFFSET_B       ((uint16_t)1 << 15)

//
static void KeyBoard_bhv(void){
    if (Gimbal.RC->state.Global_Status == Follow_Independent) {
        Arms_Drive(Gimbal.Graps->TD_t, 0, 0, 0, 0, 0,1);
    }
}
