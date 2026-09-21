#ifndef __DATA_PROCESSING_H__
#define __DATA_PROCESSING_H__

#include "FreeRTOS.h"
#include "queue.h"
#include "string.h"
#include "ins.h"
#include "Init.h"
#include "hero_shoot.h"
#include "referee.h"
#include "config.h"

#ifdef chassis_board
/*按键宏定义*/
#define RECEIVE_KEY_W             (received_data.rc_ctrl.key & (1 << 0))
#define RECEIVE_KEY_S             (received_data.rc_ctrl.key & (1 << 1))
#define RECEIVE_KEY_A             (received_data.rc_ctrl.key & (1 << 2))
#define RECEIVE_KEY_D             (received_data.rc_ctrl.key & (1 << 3))
#define RECEIVE_KEY_SHIFT         (received_data.rc_ctrl.key & (1 << 4))
#define RECEIVE_KEY_CTRL          (received_data.rc_ctrl.key & (1 << 5))
#define RECEIVE_KEY_Q             (received_data.rc_ctrl.key & (1 << 6))
#define RECEIVE_KEY_E             (received_data.rc_ctrl.key & (1 << 7))
#define RECEIVE_KEY_R             (received_data.rc_ctrl.key & (1 << 8))
#define RECEIVE_KEY_F             (received_data.rc_ctrl.key & (1 << 9))
#define RECEIVE_KEY_G             (received_data.rc_ctrl.key & (1 << 10))
#define RECEIVE_KEY_Z             (received_data.rc_ctrl.key & (1 << 11))
#define RECEIVE_KEY_X             (received_data.rc_ctrl.key & (1 << 12))
#define RECEIVE_KEY_C             (received_data.rc_ctrl.key & (1 << 13))
#define RECEIVE_KEY_V             (received_data.rc_ctrl.key & (1 << 14))
#define RECEIVE_KEY_B             (received_data.rc_ctrl.key & (1 << 15))

//左右拨杆
#define received_left_switch     received_data.rc_ctrl.rc.s[0]
//侧拨杆
#define received_side_switch     received_data.rc_ctrl.rc.ch[4]
//侧拨杆状态
#define received_side_switch_on   (received_side_switch >= 600)    //侧拨杆开
#define received_side_switch_off  (received_side_switch <= -600)   //侧拨杆关

#endif

//双板通信数据
#pragma pack(1)          // 1 字节对齐
typedef struct 
{
    uint8_t sbus_online;
    float ins_yaw;
    #ifdef COMPILE_HERO_GIMBAL
        float yaw_position;
    #endif
    #if defined(COMPILE_AGV_CHASSIS) || defined(COMPILE_ARM)
        
    #endif 
    RC_ctrl_t rc_ctrl;
    uint16_t crc16;             // CRC16校验码
} board_data;
#pragma pack()

//发送视觉通信结构体
#pragma pack(1)
typedef struct
{
  uint8_t head[2];
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序，四元数
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  uint8_t tail;
} GimbalToVision;
#pragma pack()

//上位机上行聚合快照：电控自取数据填进本结构体，
//周期任务里 HostReport_Send() 会把它按新协议拆成 B1/B2/B4 帧并经 CDC 发送(B3 直接取链路快照)
typedef struct
{
    /* B1 GimbalState 云台姿态 */
    float   gimbal_q[4];     // 四元数 wxyz
    float   gimbal_yaw;      // 云台绝对角 rad，左正
    float   gimbal_pitch;    // 云台绝对角 rad，抬头为负
    /* B4 ChassisState 底盘实测反馈(须同一采样周期) */
    float   chassis_vx;      // 底盘系前向实测 m/s
    float   chassis_vy;      // 底盘系左向实测 m/s
    float   chassis_w;       // 底盘实测逆时针 rad/s
    float   yaw_odom;        // 云台相对底盘编码器角，与速度同采样时刻
    uint8_t chassis_flags;   // bit0=本周期发生功率限幅，其余为 0
    /* B2 ShootState 射击状态 */
    float   bullet_speed;    // 弹速 m/s
    uint16_t bullet_count;   // 累计实际发射数，自然回绕
} host_report_t;

extern host_report_t host_report;

/** 按车型宏填充上行数据(目前为空实现，见 data_processing.c 内各 #ifdef 分支) */
void HostReport_Collect(host_report_t *r);
/** 周期调用：把 host_report 拆成 B1/B2/B4 + B3(链路) 组帧 usb_send */
void HostReport_Send(uint32_t now_ms);


typedef enum
{
    Rising_angle = 5,             // 抬升电机
    ARM_Motor1 = 2,               // 机械臂1号电机(旋转)
    ARM_Motor2 = 1,               // 机械臂2号电机(小臂)
    ARM_Motor3 = 0,               // 机械臂3号电机(大臂)
    Claw_updown = 3,              // 夹爪上下自由度
    Claw_clawmp = 4               // 夹爪夹紧自由度
} User_control_data;

extern uint16_t motor_angle[6];
extern board_data received_data;

void data_task(void *argument);
#ifdef gimbal_board
void Board_data_Init(board_data *sent_data);
#endif

#if defined(COMPILE_GIMBAL) || defined(COMPILE_UAV_GIMBAL) || defined(ROBOTIC_SWING_GIMBAL) || defined(COMPILE_HERO_GIMBAL)
void Gimbal_sent_vision(GimbalToVision *sent_vision, INS_t *ins);
#endif

typedef struct
{
    uint32_t total;    // 所有 CDC 包（含格式错误）
    uint32_t vision;   // VisionToGimbal 有效包
    uint32_t auto_fb;  // auto_feedback 有效包
    float    rate_hz;  // 上一统计周期内 vision 包频率（Hz）
} CDC_RxStats_t;

extern CDC_RxStats_t cdc_stats;
void CDC_RxStats_Get(CDC_RxStats_t *stats);

typedef struct
{
    uint32_t total;    // UART5 所有触发回调的包数（含丢帧）
    uint32_t valid;    // sbus_online == 1 时的有效包数
    float    rate_hz;  // 上一统计周期内有效包频率（Hz）
} SBUS_RxStats_t;

extern SBUS_RxStats_t sbus_stats;
void SBUS_RxStats_Get(SBUS_RxStats_t *stats);

#endif
