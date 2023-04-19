#ifndef __REFEREE_H_
#define __REFEREE_H_
#include "main.h"




/*****************referee data part*****************/

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;


//比赛状态数据：0x0001。发送频率：1Hz，发送范围：所有机器人。
typedef __packed struct //0001
{
    uint8_t game_type : 4;      //0-3bit 1机甲大师赛,2单项,3 ICRA, 4 3v3联盟赛 ,5 联盟赛1v1
    uint8_t game_progress : 4;  //4-7bit 当前比赛阶段 0未开始 , 1准备 2自检 3 5s倒计时 4对战中 5比赛结算中
    uint16_t stage_remain_time; //当前阶段剩余时间，单位 s
    uint64_t SyncTimeStamp;     //Unix精确时间 当机载端收到有效的 NTP 服务器授时后生效
} ext_game_state_t;


//比赛结果数据：0x0002。发送频率：比赛结束后发送，发送范围：所有机器人。
typedef __packed struct //0002
{
    uint8_t winner; //0平局 1红 2蓝
} ext_game_result_t;


//机器人血量数据：0x0003。发送频率：1Hz，发送范围：所有机器人。
typedef __packed struct
{
    uint16_t red_1_robot_HP;//红 1 英雄机器人血量，未上场以及罚下血量为 0
    uint16_t red_2_robot_HP;//工程
    uint16_t red_3_robot_HP;//步兵
    uint16_t red_4_robot_HP;//步兵
    uint16_t red_5_robot_HP;//步兵
    uint16_t red_7_robot_HP;//哨兵
//    uint16_t red_outpost_HP;

    uint16_t red_base_HP;
    //TODO:合理怀疑少了前哨站
    uint16_t blue_1_robot_HP;//蓝 1 英雄机器人血量，未上场以及罚下血量为 0
    uint16_t blue_2_robot_HP;//工程
    uint16_t blue_3_robot_HP;//步兵
    uint16_t blue_4_robot_HP;//步兵
    uint16_t blue_5_robot_HP;//步兵
    uint16_t blue_7_robot_HP;//哨兵
//    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;


//场地事件数据 发送频率：1Hz 周期发送，发送范围：己方机器人。
typedef __packed struct //0101
{
  /*
   * bit0 己方补给站 1 号补血点占领状态 1 为已占领；
   *    1 己方补给站 2 号补血点占领状态 1 为已占领;
   *    2 己方补给站 3 号补血点占领状态 1 为已占领；
   *    3-5 己方能量机关状态：
   *    3 为打击点占领状态，1 为占领；
   *    4 为小能量机关激活状态，1 为已激活；
   *    5 为大能量机关激活状态，1 为已激活；
   *    6 己方侧 R2/B2 环形高地占领状态 1 为已占领；
   *    7 己方侧 R3/B3 梯形高地占领状态 1 为已占领；
   *    8 己方侧 R4/B4 梯形高地占领状态 1 为已占领；
   *    9 己方基地护盾状态： 1有 0无
   *    10己方前哨战状态：1活 0嘎
   * */
    uint32_t event_type;
} ext_event_data_t;


//TODO:不知道23赛季还有没有补单这个玩意，但是买东西应该要的吧
//补给站动作标识：0x0102。发送频率：动作触发后发送，发送范围：己方机器人。
typedef __packed struct //0x0102
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


typedef __packed struct //0x0103
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

//裁判警告信息：cmd_id (0x0104)。发送频率：警告发生后发送，发送范围：己方机器人。
typedef __packed struct
{
    uint8_t level;  //警告等级 1黄 2红 3判负
    uint8_t foul_robot_id;//犯规机器人ID
} ext_referee_warning_t;

//比赛机器人状态：0x0201。发送频率：10Hz
typedef __packed struct //0x0201
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_heat0_cooling_rate;
    uint16_t shooter_heat0_cooling_limit;
    uint16_t shooter_heat1_cooling_rate;
    uint16_t shooter_heat1_cooling_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

typedef __packed struct //0x0202
{
    uint16_t chassis_volt;    //底盘电压 毫伏
    uint16_t chassis_current; //底盘电流 毫安
    float chassis_power;      //
    uint16_t chassis_power_buffer;
    uint16_t shooter_heat0;
    uint16_t shooter_heat1;
} ext_power_heat_data_t;

//
typedef __packed struct //0x0203
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

typedef __packed struct //0x0204
{
    uint8_t power_rune_buff;
} ext_buff_musk_t;

typedef __packed struct //0x0205
{
    uint8_t energy_point;
    uint8_t attack_time;
} aerial_robot_energy_t;


typedef __packed struct //0x0206
{
    uint8_t armor_type : 4; //当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0
    uint8_t hurt_type : 4;  //血量变化类型

} ext_robot_hurt_t;

typedef __packed struct //0x0207
{
    uint8_t bullet_type;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;


//机器人 RFID 状态：0x0209。发送频率：1Hz，发送范围：单一机器人
typedef __packed struct
{
  uint32_t rfid_status; //bit 7：工程机器人复活卡 RFID 状态；
} ext_rfid_status_t;


typedef __packed struct
{
    uint8_t bullet_remaining_num;
} ext_bullet_remaining_t;

typedef __packed struct //0x0301
{
    uint16_t send_ID;
    uint16_t receiver_ID;
    uint16_t data_cmd_id;
    uint16_t data_len;
    uint8_t *data;
} ext_student_interactive_data_t;

typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;


typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;

//裁判系统主结构体
typedef struct {
  ext_game_robot_HP_t *robotHp;
  ext_event_data_t *extEventData;//当工程需要指挥时可能要加
  ext_power_heat_data_t *powerHeatData;
  ext_game_robot_pos_t *robotPos;
  ext_rfid_status_t *rfidStatus;//RFID状态
}Referee_t;


/**************function declaration*************/

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

#endif

