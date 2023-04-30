
/***
 * ┌───┐   ┌───┬───┬───┬───┐ ┌───┬───┬───┬───┐ ┌───┬───┬───┬───┐ ┌───┬───┬───┐
 * │Esc│   │ F1│ F2│ F3│ F4│ │ F5│ F6│ F7│ F8│ │ F9│F10│F11│F12│ │P/S│S L│P/B│  ┌┐    ┌┐    ┌┐
 * └───┘   └───┴───┴───┴───┘ └───┴───┴───┴───┘ └───┴───┴───┴───┘ └───┴───┴───┘  └┘    └┘    └┘
 * ┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───────┐ ┌───┬───┬───┐ ┌───┬───┬───┬───┐
 * │~ `│! 1│@ 2│# 3│$ 4│% 5│^ 6│& 7│* 8│( 9│) 0│_ -│+ =│ BacSp │ │Ins│Hom│PUp│ │N L│ / │ * │ - │
 * ├───┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─────┤ ├───┼───┼───┤ ├───┼───┼───┼───┤
 * │ Tab │ Q │ W │ E │ R │   │   │ U │ I │ O │ P │{ [│} ]│ | \ │ │Del│End│PDn│ │ 7 │ 8 │ 9 │   │
 * ├─────┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴─────┤ └───┴───┴───┘ ├───┼───┼───┤ + │
 * │      │ A │ S │ D │ F │ G │   │ J │ K │ L │: ;│" '│ Enter  │               │ 4 │ 5 │ 6 │   │
 * ├──────┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴────────┤     ┌───┐     ├───┼───┼───┼───┤
 * │ Shift  │ Z │ X │ C │ V │ B │   │ M │< ,│> .│? /│  Shift   │     │ ↑ │     │ 1 │ 2 │ 3 │   │
 * ├─────┬──┴─┬─┴──┬┴───┴───┴───┴───┴───┴──┬┴───┼───┴┬────┬────┤ ┌───┼───┼───┐ ├───┴───┼───┤ E││
 * │ Ctrl│    │Alt │         Space         │ Alt│    │    │Ctrl│ │ ← │ ↓ │ → │ │   0   │ . │←─┘│
 * └─────┴────┴────┴───────────────────────┴────┴────┴────┴────┘ └───┴───┴───┘ └───────┴───┴───┘
 */
 //total 16 keys
//机械臂需要：一键到位、微操、一键归零
//图传需要：跟随、自己动、看储矿
//救援抓需要：一键到位
//整车：只用wasd或者键鼠
//储矿需要翻
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
#include "RemoteDeal.h"
#include "maths.h"
/*遥控结构体*/
REMOTE_t REMOTE;
///*遥控死区*/
//static const uint8_t DeadZone = 10;

/*************************************************************************************************
*名称:	Get_RemoteDeal_Point
*功能:	返回处理后的遥控数值控制变量，通过指针传递方式传递信息
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
REMOTE_t *Return_RemoteDeal_Point(void)
{
    return &REMOTE;
}

/*************************************************************************************************
*名称:	Remote_Data_Zero
*功能:	遥控数值清零
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
static void Remote_Data_Zero(void)
{
    RC_DataReload();//遥控器数据清零

    /*状态值初始化 全部暂停*/
    REMOTE.state.Global_Status = 0;
}

/*************************************************************************************************
*名称:	Remote_Data_Init
*功能:	遥控数值处理初始化
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
void Remote_Data_Init(void)
{
    /*遥控数值清零*/
    Remote_Data_Zero();
    REMOTE.RC_ctrl->rc.s1 = REMOTE.RC_ctrl->rc.s2 = 2; //此键位为断电键位

    /*低通滤波初始化*/
    first_order_filter_init(&REMOTE.RC_X, 0.08);
    first_order_filter_init(&REMOTE.RC_Y, 0.08);
    first_order_filter_init(&REMOTE.RC_Z, 0.08);

    first_order_filter_init(&REMOTE.KM_X, 0.08);
    first_order_filter_init(&REMOTE.KM_Y, 0.08);
    first_order_filter_init(&REMOTE.KM_Z, 0.08);


    /*获取遥控指针*/
    REMOTE.RC_ctrl = RC_Get_RC_Pointer();
    //dma双缓冲遥控器协议接收初始化
    ECF_RC_Init();
}

/*************************************************************************************************
*名称:	Rc_Deal
*功能:	处理遥杆拨杆的值
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
static void Rc_Deal(void)
{
    /*滤波处理*/
    first_order_filter(&REMOTE.RC_X, REMOTE.RC_ctrl->rc.ch[3]);
    first_order_filter(&REMOTE.RC_Y, REMOTE.RC_ctrl->rc.ch[2]);
    first_order_filter(&REMOTE.RC_Z, -REMOTE.RC_ctrl->rc.ch[0]);
}

/*************************************************************************************************
*名称:	KEY_MOUSE
*功能:	处理键盘鼠标的值
*形参: 	无
*返回:	无
*说明:	分支结构可以优化
*************************************************************************************************/
static void Key_Mouse_Deal(void)
{

    uint16_t key =REMOTE.RC_ctrl->key.v;
    int16_t ws[2]={0,0};
    int16_t ad[2]={0,0};

    //整车状态切换
    if (key & KEY_PRESSED_OFFSET_G) {
        REMOTE.state.Global_Status = 1 - REMOTE.state.Global_Status;
    }

    /*************************************底盘前后*********************************/
    if (key & KEY_PRESSED_OFFSET_W)
    {
        //底盘独立，可以控制速度
        if (REMOTE.state.Global_Status == Follow_Independent) {
            if (key & KEY_PRESSED_OFFSET_SHIFT && key & KEY_PRESSED_OFFSET_W)
            {
                ws[0]=6000;
            }
            else if(key & KEY_PRESSED_OFFSET_CTRL && key & KEY_PRESSED_OFFSET_W)
            {
                ws[0]=200;
            }
            else
            {
                ws[0]=3000;
            }
        }
        //底盘其他模式
        else{
            ws[0]=3000;
        }

    }
    if (key & KEY_PRESSED_OFFSET_S)
    {
        if (REMOTE.state.Global_Status == Follow_Independent){
            if (key & KEY_PRESSED_OFFSET_SHIFT && key & KEY_PRESSED_OFFSET_S)
            {
                ws[1]=-6000;
            }
            else if(key & KEY_PRESSED_OFFSET_CTRL && key & KEY_PRESSED_OFFSET_S)
            {
                ws[1]=-200;
            }
            else
            {
                ws[1]=-3000;
            }
        }
        else{
            ws[1]=-3000;
        }

    }
    REMOTE.RC_ctrl->key.kv0=ws[0]+ws[1];


    /**************************************底盘左右*********************************/
    if (key & KEY_PRESSED_OFFSET_A)
    {
        if (REMOTE.state.Global_Status == Follow_Independent){
            if (key & KEY_PRESSED_OFFSET_SHIFT && key & KEY_PRESSED_OFFSET_A)
            {
                ad[0]=-6000;
            }
            else if(key & KEY_PRESSED_OFFSET_CTRL && key & KEY_PRESSED_OFFSET_A)
            {
                ad[0]=-200;
            }
            else
            {
                ad[0]=-3000;
            }
        }
        else
        {
            ad[0]=-3000;
        }
    }

    if (key & KEY_PRESSED_OFFSET_D)
    {
        if (REMOTE.state.Global_Status == Follow_Independent){
            if (key & KEY_PRESSED_OFFSET_SHIFT && key & KEY_PRESSED_OFFSET_D)
            {
                ad[1]=6000;
            }
            else if(key & KEY_PRESSED_OFFSET_CTRL && key & KEY_PRESSED_OFFSET_D)
            {
                ad[1]=200;
            }
            else
            {
                ad[1]=3000;
            }
        }
        else{
            ad[1]=3000;
        }

    }
    REMOTE.RC_ctrl->key.kv1=ad[0]+ad[1];

    /*************************************鼠标X轴************************************/
    REMOTE.RC_ctrl->mouse.x *= 5.0f;
    /**************************************鼠标Y轴***********************************/
    REMOTE.RC_ctrl->mouse.y *= 5.0f;

//    /*******************************自动夹取********************************/
//    if (key & KEY_PRESSED_OFFSET_Q)
//    {
//            REMOTE.state.Grasp_Up = 1; // 自动夹取
//    }
//
//    /********************************兑换抬升*******************************/
//    if (key & KEY_PRESSED_OFFSET_E)
//    {
//            REMOTE.state.Grasp_Up = 2; // 自动夹取
//    }
//
//    /********************************翻转复位*******************************/
//    if (key & KEY_PRESSED_OFFSET_R)
//    {
//            REMOTE.state.Grasp_Up = 3; // 自动夹取
//    }
//
//    /*********************************RFID********************************/
//    if (key & KEY_PRESSED_OFFSET_F)
//    {
//
//        if (key & KEY_PRESSED_OFFSET_F && key & KEY_PRESSED_OFFSET_CTRL)
//        {
//            REMOTE.state.Gimbal_Yaw = 1;
//        }
//        else
//        {
//            REMOTE.state.Gimbal_Yaw = 0;
//        }
//    }
//
//    /*********************************救援爪********************************/
//    if (key & KEY_PRESSED_OFFSET_G)
//    {
//
//        if (key & KEY_PRESSED_OFFSET_G && key & KEY_PRESSED_OFFSET_CTRL)
//        {
//						REMOTE.state.RFID = 0; //RFID缩
//            REMOTE.state.Barrier = 0; //救援爪缩
//        }
//        else
//        {
//						REMOTE.state.RFID = 1; //RFID伸
//            REMOTE.state.Barrier = 1; //救援爪伸
//        }
//    }
//
//    /* ========================================以下为底盘====================================== */
//
//    /*********************************正常**********************************/
//    if (key & KEY_PRESSED_OFFSET_SHIFT)
//    {
//
//        REMOTE.state.Independent = 1; //底盘正常运动
//        REMOTE.state.Rotation = 2;    //小陀螺模式暂停
//        REMOTE.state.Wiggle = 2;      //扭腰模式暂停
//    }
//
//    /*********************************大陀螺**********************************/
//    if (key & KEY_PRESSED_OFFSET_Z)
//    {
//
//        REMOTE.state.Independent = 2; //底盘正常暂停
//        REMOTE.state.Rotation = 1;    //大陀螺模式
//        REMOTE.state.Wiggle = 2;      //扭腰模式暂停
//    }
//
//    /* ======================================以上为底盘===================================== */
//    /**********************************松夹**********************************/
//    if (key & KEY_PRESSED_OFFSET_X)
//    {
//        if (key & KEY_PRESSED_OFFSET_X && key & KEY_PRESSED_OFFSET_CTRL)
//        {
//						REMOTE.state.Translation = 0;  //上装复位关闭
//        }
//        else
//        {
//						REMOTE.state.Translation = 1;  //上装复位关闭
//				}
//    }
//
//    /*******************************兑换一箱********************************/
//    if (key & KEY_PRESSED_OFFSET_C)
//    {
//        if (key & KEY_PRESSED_OFFSET_C && key & KEY_PRESSED_OFFSET_CTRL)
//        {
//						REMOTE.state.Telescoping = 0;  //上装复位关闭
//        }
//        else
//        {
//						REMOTE.state.Telescoping = 1;  //上装复位关闭
//				}
//    }
//
//    /********************************兑换二箱********************************/
//    if (key & KEY_PRESSED_OFFSET_V)
//    {
//        if (key & KEY_PRESSED_OFFSET_V && key & KEY_PRESSED_OFFSET_CTRL)
//        {
//						REMOTE.state.Clap = 0;  //上装复位关闭
//        }
//        else
//        {
//						REMOTE.state.Clap = 1;  //上装复位关闭
//				}
//    }
//
//    /********************************兑换三箱********************************/
//    if (key & KEY_PRESSED_OFFSET_B)
//    {
//        if (key & KEY_PRESSED_OFFSET_B && key & KEY_PRESSED_OFFSET_CTRL)
//        {
//						REMOTE.state.Flip = 0;  //上装复位关闭
//        }
//        else
//        {
//						REMOTE.state.Flip = 1;  //上装复位关闭
//				}
//    }

    /*限制幅度处理*/
//    REMOTE.RC_ctrl->key.kv0 = limit(REMOTE.RC_ctrl->key.kv0, 660, -660);
//    REMOTE.RC_ctrl->key.kv1 = limit(REMOTE.RC_ctrl->key.kv1, 660, -660);
//    REMOTE.RC_ctrl->mouse.x = limit(REMOTE.RC_ctrl->mouse.x, 660, -660);
//    REMOTE.RC_ctrl->mouse.y = limit(REMOTE.RC_ctrl->mouse.y, 660, -660);
    /*滤波处理*/
//    First_Order(&REMOTE.KM_X, REMOTE.RC_ctrl->key.kv0); //底盘左右
//    First_Order(&REMOTE.KM_Y, REMOTE.RC_ctrl->key.kv1); //底盘前后
    // First_Order(&REMOTE.KM_Z, REMOTE.RC_ctrl->mouse.x); //鼠标x轴


}

/*************************************************************************************************
*名称:	RC_DATA_DEAL
*功能:	遥控数值处理
*形参: 	无
*返回:	无
*说明:	无
*************************************************************************************************/
void Remote_Data_Deal(void)
{
//    static unsigned char _s1 = 0;

//    if (_s1 != REMOTE.RC_ctrl->rc.s1)
//        Remote_Data_Zero();

//    if (REMOTE.RC_ctrl->Flag)
//    {
//        if (REMOTE.RC_ctrl->rc.s1 == 3)
//        {
//            Key_Mouse_Deal();
//        }
//        else
//        {
//            Rc_Deal();
//        }

//        REMOTE.RC_ctrl->Flag = 0;
//    }

//    _s1 = REMOTE.RC_ctrl->rc.s1;
	if(REMOTE.RC_ctrl->rc.s1==1&&REMOTE.RC_ctrl->rc.s2==1)
	{
		Key_Mouse_Deal();
        detect_hook(DBUS_TOE);//记录在线时间
	}
	else{
		Rc_Deal();
        detect_hook(DBUS_TOE);
	}
		
	
}

