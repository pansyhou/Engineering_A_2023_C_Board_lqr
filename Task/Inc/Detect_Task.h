#ifndef __DETECT_TASK_H
#define __DETECT_TASK_H


#include "struct_typedef.h"

#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10


//错误码以及对应设备顺序
enum errorList
{
    DBUS_TOE = 0,
    YAW_GIMBAL_MOTOR_TOE,
    ROLL_GIMBAL_MOTOR_TOE,
    PITCH_GIMBAL_MOTOR_TOE,
    LIFT_GIMBAL_MOTOR_TOE,
    ERROR_LIST_LENGHT,
};


//设备错误结构体
typedef __packed struct
{
    //注意，以下时间全是FreeRTOS的时间戳
    uint32_t new_time;      //在钩子函数更新的时间
    uint32_t last_time;     //记录的上一次时间
    uint32_t lost_time;     //掉线时间
    uint32_t work_time;     //掉线重连后的工作时间
    uint16_t set_offline_time : 12;     //所设置的掉线时间,12指的是占12个bit
    uint16_t set_online_time : 12;      //所设置的上线时间
    uint8_t enable : 1;         //是否使能
    uint8_t priority : 4;       //该设备的优先级
    uint8_t error_exist : 1;    //解决函数解决不了的问题就设1
    uint8_t is_lost : 1;        //是否掉线
    uint8_t data_is_error : 1;  //

    fp32 frequency;
    bool_t (*data_is_error_fun)(void);  //数值错误判断函数
    void (*solve_lost_fun)(void);       //问题解决处理函数
    void (*solve_data_error_fun)(void); //数值错误处理函数
} error_t;

extern void Detect_Task(void const *pvParameters);
extern bool_t toe_is_error(uint8_t err);
extern void detect_hook(uint8_t toe);
extern const error_t *get_error_list_point(void);
#endif

