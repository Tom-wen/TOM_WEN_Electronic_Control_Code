#ifndef __REFEREE_H__
#define __REFEREE_H__

#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "bsp_usart.h"
#include "CRC.h"
#include "string.h"
#include "remote_control.h"
#include "Circular_buffer.h"

//命令码定义
#define CMD_ID_ROBOT_STATUS             0x0201   // 机器人性能体系数据
#define CMD_ID_POWER_HEAT               0x0202   // 实时底盘缓冲能量和射击热量数据
#define CMD_ID_ROBOT_POS                0x0203   // 机器人位置数据
#define CMD_ID_BUFF                     0x0204   // 机器人增益和底盘能量数据
#define CMD_ID_HURT                     0x0206   // 伤害状态数据
#define CMD_ID_SHOOT                    0x0207   // 弹速
#define CMD_ID_PROJECTILE_ALLOWANCE     0x0208   // 允许发弹量数据
#define CMD_ID_RFID                     0x0209   // RFID模块状态数据
#define CMD_ID_GROUND_ROBOT_POS         0x020B   // 地面机器人位置数据
#define CMD_ID_SENTRY_INFO              0x020D   // 哨兵自主决策信息同步数据
#define CMD_ID_REMOTE_CONTROL           0x0304   // 键鼠遥控数据

//机器人性能状态(0x0201)
typedef struct 
{
    uint8_t robot_id; 
    uint8_t robot_level; 
    uint16_t current_HP; 
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value; 
    uint16_t shooter_barrel_heat_limit; 
    uint16_t chassis_power_limit; 
    uint8_t power_management_gimbal_output : 1; 
    uint8_t power_management_chassis_output : 1; 
    uint8_t power_management_shooter_output : 1;
}robot_state;

//0x0202 - 实时底盘缓冲能量和射击热量数据（14字节）
typedef struct 
{ 
    uint16_t reserved_1;      // 保留位
    uint16_t reserved_2;      // 保留位
    float reserved_3;         // 保留位
    uint16_t buffer_energy;  // 缓冲能量（单位：J）
    uint16_t shooter_17mm_1_barrel_heat;  // 第1个17mm发射机构的射击热量
    uint16_t shooter_42mm_barrel_heat;   // 42mm发射机构的射击热量
} power_heat_data_t;


//0x0203 - 机器人位置数据（12字节）
typedef struct 
{ 
    float x;      // 本机器人位置x坐标，单位：m
    float y;      // 本机器人位置y坐标，单位：m
    float angle;   // 本机器人测速模块的朝向，单位：度，正北为0度
} robot_pos_t;


//0x0204 - 机器人增益和底盘能量数据
typedef struct 
{ 
    uint8_t recovery_buff;       // 机器人回血增益（百分比，值为10表示每秒恢复血量上限的10%）
    uint16_t cooling_buff;       // 机器人射击热量冷却增益具体值（直接值，值为x表示热量冷却增加x/s）
    uint8_t defence_buff;       // 机器人防御增益（百分比，值为50表示50%防御增益）
    uint8_t vulnerability_buff;  // 机器人负防御增益（百分比，值为30表示-30%防御增益）
    uint16_t attack_buff;       // 机器人攻击增益（百分比，值为50表示50%攻击增益）
    uint8_t remaining_energy;    // 机器人剩余能量值反馈（bit位定义见文档）
} buff_t;

//0x0206 - 伤害状态数据（1字节）
typedef struct 
{ 
    uint8_t armor_id : 4;        // 装甲模块ID编号（0表示其他原因）
    uint8_t HP_deduction_reason : 4;  // 血量变化类型（0：被弹丸攻击，1：模块离线，5：被撞击）
} hurt_data_t;

//0x0207- 实时射击数据（7字节）
typedef struct 
{ 
    uint8_t bullet_type;        // 弹丸类型（bit1：17mm，bit2：42mm）
    uint8_t shooter_number;      // 发射机构ID（1：17mm，3：42mm）
    uint8_t launching_frequency; // 弹丸射速（Hz）
    float initial_speed;         // 弹丸初速度（m/s）
} shoot_data_t;

//0x0208 - 允许发弹量数据（8字节）
typedef struct 
{ 
    uint16_t projectile_allowance_17mm;   // 17mm弹丸允许发弹量
    uint16_t projectile_allowance_42mm;   // 42mm弹丸允许发弹量
    uint16_t remaining_gold_coin;          // 剩余金币数量
    uint16_t projectile_allowance_fortress; // 堡垒增益点提供的储备17mm弹丸允许发弹量
} projectile_allowance_t;


//0x0209 - RFID模块状态数据（5字节）
typedef struct 
{ 
    uint32_t rfid_status;   // RFID卡检测状态（32位bit位对应不同增益点）
    uint8_t rfid_status_2;  // 隧道相关RFID状态（2位bit位）
} rfid_status_t;

//0x020B - 地面机器人位置数据（40字节）
typedef struct 
{ 
    float hero_x;        // 己方英雄机器人位置x轴坐标（m）
    float hero_y;        // 己方英雄机器人位置y轴坐标（m）
    float engineer_x;     // 己方工程机器人位置x轴坐标（m）
    float engineer_y;     // 己方工程机器人位置y轴坐标（m）
    float standard_3_x;   // 己方3号步兵机器人位置x轴坐标（m）
    float standard_3_y;   // 己方3号步兵机器人位置y轴坐标（m）
    float standard_4_x;   // 己方4号步兵机器人位置x轴坐标（m）
    float standard_4_y;   // 己方4号步兵机器人位置y轴坐标（m）
    float reserved_1;     // 保留位
    float reserved_2;     // 保留位
} ground_robot_position_t;

//0x020D - 哨兵自主决策信息同步数据（6字节）
typedef struct 
{ 
    uint32_t sentry_info;   // 哨兵信息（bit位定义包括兑换信息、复活、姿态等）
    uint16_t sentry_info_2; // 哨兵姿态和能量机关状态信息
} sentry_info_t;

//图传键鼠数据
typedef struct 
{ 
    int16_t mouse_x; 
    int16_t mouse_y; 
    int16_t mouse_z; 
    uint8_t left_button_down; 
    uint8_t right_button_down; 
    uint16_t keyboard_value; 
    uint16_t reserved; 
}remote_control_t; 

//裁判系统数据汇总
typedef struct 
{
    robot_state robot;
    shoot_data_t shoot_data;
    remote_control_t referee_remote;
    power_heat_data_t power_heat;
    robot_pos_t robot_pos;
    buff_t buff;
    hurt_data_t hurt;
    projectile_allowance_t projectile_allowance;
    rfid_status_t rfid_status;
    ground_robot_position_t ground_robot_pos;
    sentry_info_t sentry_info;    
}referee_receive;


void referee_usart_task(void *argument);
void referee_usart_analyse(REFEREE_RX_TypeDef *pRecvUartData, referee_receive *referee_data);
void referee_routine_analyse(referee_receive *referee_data);

#endif
