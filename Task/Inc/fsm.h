#ifndef __FSM_H_
#define __FSM_H_
#include "struct_typedef.h"

#ifndef NULL
	#define NULL 0
#endif

#define State_Line 		3   //s行和列的范围
#define State_Column 	3

/*状态结构体*/
typedef struct _state
{
    void (*State_Prepare)(void);       //准备状态函数指针（用于执行该状态的准备函数
    void (*State_Process)(void);       //状态执行函数指针（一般这个指向下面的行为
    void (*Behavior_Process)(void);    //正常行为函数指针（指向目前行为的驱动
}State_t;

/*有限状态机*/
typedef struct _fsm
{
    uint8_t (*State_Change)(State_t *L_Sta,State_t *C_Sta); //状态变更函数
    State_t *Last_State;                                    //上次状态
    State_t *Current_State;                                 //目前状态
    State_t (*State_Table)[State_Column];                   //状态参数表
}FSM_t;



/*状态机处理函数*/
void FSM_Deal(FSM_t *fsm, uint8_t s1, uint8_t s2);
/*状态变更函数*/
uint8_t StateChange(State_t *L_Sta,State_t *C_Sta);
#endif
