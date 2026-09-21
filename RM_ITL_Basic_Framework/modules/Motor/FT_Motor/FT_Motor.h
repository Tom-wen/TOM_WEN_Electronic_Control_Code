#ifndef __FT_MOTOR_H__
#define __FT_MOTOR_H__

#include "main.h"

/* ===================== STS/SMS舵机内存表寄存器地址 ===================== */

/* EPROM(读写) */
#define FT_REG_ID               5
#define FT_REG_BAUD_RATE        6
#define FT_REG_MODE             33  /* 0=位置伺服 1=恒速电机 2=步进电机 */

/* SRAM(读写) */
#define FT_REG_TORQUE_ENABLE    40  /* 0=松开 1=锁紧 */
#define FT_REG_ACC              41
#define FT_REG_GOAL_POS_L       42
#define FT_REG_GOAL_TIME_L      44  /* 力位模式下为目标扭矩 */
#define FT_REG_GOAL_SPEED_L     46
#define FT_REG_LOCK             55

/* SRAM(只读) */
#define FT_REG_PRESENT_POS_L    56
#define FT_REG_PRESENT_SPEED_L  58
#define FT_REG_PRESENT_LOAD_L   60
#define FT_REG_PRESENT_VOLTAGE  62
#define FT_REG_PRESENT_TEMP     63
#define FT_REG_MOVING           66
#define FT_REG_PRESENT_CURRENT_L 69

/* ===================== 协议常量 ===================== */

#define FT_BROADCAST_ID         0xFE
#define FT_INST_PING            0x01
#define FT_INST_READ            0x02
#define FT_INST_WRITE           0x03
#define FT_INST_REG_WRITE       0x04
#define FT_INST_REG_ACTION      0x05
#define FT_INST_SYNC_WRITE      0x83

/* 工作模式 */
#define FT_MODE_SERVO           0
#define FT_MODE_MOTOR           1
#define FT_MODE_PWM             2

/* ===================== 配置 ===================== */

#define MAX_FT_SERVOS           10
#define FT_TX_BUF_SIZE          128

/* 一次读取的反馈数据长度: pos(2)+speed(2)+load(2)+volt(1)+temp(1)+?(2)+moving(1)+?(2)+current(2) = 15 */
#define FT_FEEDBACK_DATA_LEN    15

/* ===================== 数据结构 ===================== */

typedef struct {
    int16_t position;       /* 0~4095 (位置伺服模式) 位置*/
    int16_t speed;          /* 有符号, bit15=方向 速度*/
    int16_t load;           /* 有符号, bit15=方向 负载*/
    uint8_t voltage;        /* 单位: 0.1V 电压*/
    uint8_t temperature;    /* 单位: °C 温度*/
    int16_t current;        /* 有符号, 单位: mA */
    uint8_t moving;         /* 0=静止 1=运动中 */
    uint8_t online;         /* 1=收到过有效反馈 */
} FT_Feedback;

/* ===================== 全局变量 ===================== */

extern FT_Feedback ft_feedback[MAX_FT_SERVOS];

/* ===================== API ===================== */

/* 初始化, 传入舵机通信用的串口句柄 */
void FT_Motor_Init(UART_HandleTypeDef *huart);

/* ----- 位置控制 ----- */
/* 单个舵机写位置, 立即通过DMA发送 */
void FT_WritePos(uint8_t id, int16_t pos, uint16_t speed, uint8_t acc);

/* 多舵机同步写位置, 一条指令同时控制 */
void FT_SyncWritePos(const uint8_t ids[], uint8_t count,
                     const int16_t pos[], const uint16_t speed[], const uint8_t acc[]);

/* ----- 力位混合控制 ----- */
/* 单个舵机力位模式: 同时控制位置和扭矩(含使能) */
void FT_WritePosTorque(uint8_t id, int16_t pos, uint16_t torque, uint16_t speed, uint8_t acc);

/* 多舵机同步力位模式 */
void FT_SyncWritePosTorque(const uint8_t ids[], uint8_t count,
                           const int16_t pos[], const uint16_t torque[],
                           const uint16_t speed[], const uint8_t acc[]);

/* ----- 速度控制(电机模式) ----- */
void FT_WriteSpeed(uint8_t id, int16_t speed, uint8_t acc);

/* ----- 模式 & 力矩 ----- */
void FT_SetMode(uint8_t id, uint8_t mode);
void FT_TorqueEnable(uint8_t id, uint8_t enable);

/* ----- 反馈 ----- */
/* 发送读反馈指令 (从地址56读15字节) */
void FT_ReadFeedback(uint8_t id);
void FT_PollFeedback(void);
/* 解析舵机返回数据, 在UART空闲中断回调里调用 */
void FT_ParseResponse(uint8_t *data, uint16_t len);

/* 周期调用: 交替执行控制函数和轮询反馈, 解决DMA TX冲突 */
void FT_Process(void (*control_fn)(void *), void *arg);
#endif
