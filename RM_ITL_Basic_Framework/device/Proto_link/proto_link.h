#ifndef __PROTO_LINK_H__
#define __PROTO_LINK_H__
/**
  * @file       proto_link.h
  * @brief      上位机(决策/视觉) <-> 下位机(电控) 自定义通信协议，下位机一侧。
  *
  *  帧格式  :  HEADER(1) | LEN(1) | payload(n) | CRC8(1)      LEN = n + 3
  *  字节序  :  整数一律无符号小端，float 为 IEEE754 小端
  *  CRC8   :  poly=0x31, init=0xFF, MSB-first, 覆盖前 LEN-1 字节
  *
  *  下行(本侧接收) : A1 ChassisCmd / A2 GimbalCmd / A3 DecisionCmd
  *  上行(本侧发送) : B1 GimbalState / B2 ShootState / B3 LinkStatus / B4 ChassisState
  *
  *  集成示例(在 data_task 等 2ms 循环里):
  *
  *      static proto_link_t link;
  *      ProtoLink_Init(&link);                 // 上电只做一次
  *      // 任意字节源(USB CDC 回调 / 串口回调)里逐字节喂入:
  *      ProtoLink_RxPush(&link, byte);         // 环形缓冲有上界，满则溢出计数
  *      // 周期任务里解析 + watchdog + 组帧回发:
  *      uint32_t now = xTaskGetTickCount();
  *      ProtoLink_Process(&link, now);         // 解析缓冲、落结构体、刷新计数
  *      ProtoLink_Watchdog(&link, now);        // 检查三路下行超时，做故障动作
  *      ProtoLink_GetChassisCmd(&link);        // 读 A1 当前生效值
  *      proto_gimbal_state_t gs = {0};  gs.yaw = ...;  ...
  *      ProtoLink_EncodeGimbalState(tx, &gs);  // 封 B1 帧(含 CRC)，tx 足够 27 字节
  *
  *  本模块不依赖具体车型/RTOS；时间由调用方以 now_ms 传入。
  */
/**
 * 仅用于哨兵 使用，请勿用于其他用途！！！
**/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 常量 ==================== */
#define PROTO_VERSION        3u    /* B3 里的通讯版本，当前为 3 */

#define PROTO_HEAD_A1        0xA1u
#define PROTO_HEAD_A2        0xA2u
#define PROTO_HEAD_A3        0xA3u
#define PROTO_HEAD_B1        0xB1u
#define PROTO_HEAD_B2        0xB2u
#define PROTO_HEAD_B3        0xB3u
#define PROTO_HEAD_B4        0xB4u

/* 整帧总长(LEN 字段)，与逐包定义一一对应 */
#define PROTO_LEN_A1         15u
#define PROTO_LEN_A2         13u
#define PROTO_LEN_A3         5u
#define PROTO_LEN_B1         27u
#define PROTO_LEN_B2         9u
#define PROTO_LEN_B3         11u
#define PROTO_LEN_B4         20u

#define PROTO_MAX_FRAME      27u  /* 当前最大整帧(B1) */

/* 接收环形缓冲上界(字节)，需 >= 最大整帧 + 余量；取 2 的幂以用掩码 */
#define PROTO_RX_RING_SIZE   128u

/* 电控三路 watchdog */
#define PROTO_TIMEOUT_A1_MS  100u
#define PROTO_TIMEOUT_A2_MS  100u
#define PROTO_TIMEOUT_A3_MS  500u
#define PROTO_FLAG_TIMEOUT_A1 0x01u
#define PROTO_FLAG_TIMEOUT_A2 0x02u
#define PROTO_FLAG_TIMEOUT_A3 0x04u

/* ==================== 逻辑帧载荷结构体 ==================== */
/** A1 ChassisCmd 载荷 : 云台系前向/左向速度 + 底盘逆时针角速度 */
typedef struct
{
    float vx;   /* 云台系前向 m/s */
    float vy;   /* 云台系左向 m/s */
    float w;    /* 底盘逆时针 rad/s */
} proto_chassis_cmd_t;

/** A2 GimbalCmd 载荷 : 云台绝对目标角 + 使能/开火请求 */
typedef struct
{
    float yaw;              /* 绝对目标角 rad，左正 */
    float pitch;            /* 绝对目标角 rad，抬头为负 */
    uint8_t gimbal_enable;  /* 0/1 */
    uint8_t fire_request;   /* 0/1 */
} proto_gimbal_cmd_t;

/** A3 DecisionCmd 载荷 : 云台自瞄/开火独立许可 */
typedef struct
{
    uint8_t gimbal_permit;  /* 0/1 */
    uint8_t fire_permit;    /* 0/1 */
} proto_decision_cmd_t;

/** B1 GimbalState 载荷 : 云台姿态(供编码用，调用方先打好一份快照) */
typedef struct
{
    float q[4];    /* 四元数 wxyz */
    float yaw;     /* rad，左正 */
    float pitch;   /* rad，抬头为负 */
} proto_gimbal_state_t;

/** B2 ShootState 载荷 */
typedef struct
{
    float bullet_speed;    /* m/s */
    uint16_t bullet_count; /* 累计实际发射数，自然回绕 */
} proto_shoot_state_t;

/** B3 LinkStatus 载荷(链路诊断快照) */
typedef struct
{
    uint8_t proto_version;              /* 当前为 PROTO_VERSION(=3) */
    uint8_t timeout_flags;              /* bit0=A1超时 bit1=A2超时 bit2=A3超时，bit3-7=0 */
    uint16_t rx_count_a1;               /* 上行计数，自上电 0 起自然回绕 */
    uint16_t rx_count_a2;
    uint16_t rx_count_a3;
} proto_link_status_t;

/** B4 ChassisState 载荷(全部须取自同一反馈周期) */
typedef struct
{
    float vx;          /* 底盘系轮速正解实测 m/s */
    float vy;          /* 底盘系轮速正解实测 m/s */
    float w;           /* 底盘实测逆时针 rad/s */
    float yaw_odom;    /* 云台相对底盘编码器角，与速度同一采样时刻 */
    uint8_t flags;     /* bit0=本上报周期发生过功率限幅，其余为 0 */
} proto_chassis_state_t;

/* ==================== 协议实例 ==================== */
typedef struct
{
    /* —— 下行指令(解码后 / watchdog 故障动作后的"当前生效值") —— */
    proto_chassis_cmd_t  chassis_cmd;    /* A1 */
    proto_gimbal_cmd_t   gimbal_cmd;     /* A2 */
    proto_decision_cmd_t decision_cmd;   /* A3 */

    /* —— 三路 watchdog 内部状态 —— */
    uint8_t  timeout_flags;              /* 当前超时标志位(同 B3 定义) */
    bool     rx_seen[3];                 /* 该通道是否已收到过首帧合法帧 */
    uint32_t rx_last_ms[3];              /* 该通道最近一帧合法帧时刻(ms) */

    /* —— 链路诊断: 合法帧计数，自上电 0 起自然回绕 —— */
    uint16_t rx_count[3];                /* [0]=A1 [1]=A2 [2]=A3 */

    /* —— 接收/解析调试统计(不影响 watchdog 与合法帧计数) —— */
    uint32_t frame_ok;          /* 通过 头/长度/CRC/payload 校验并落库的帧数 */
    uint32_t frame_crc_err;     /* CRC 失败(触发丢 1B 重找)的次数 */
    uint32_t frame_len_err;     /* LEN 与本帧头不匹配(丢 1B 重找)的次数 */
    uint32_t frame_payload_err; /* 长度/CRC 通过但 payload 非法(如非有限 float)被拒帧数 */
    uint32_t drop_cnt;          /* 重同步过程中丢弃的总字节数 */
    uint32_t overflow_cnt;      /* 环形缓冲溢出(丢最新字节)计数 */

    /* —— 接收环形缓冲 & 读写指针(绝对位置) —— */
    uint8_t  ring[PROTO_RX_RING_SIZE];
    uint32_t head;
    uint32_t tail;
} proto_link_t;

/* ==================== CRC8 ==================== */
/**
  * @brief 计算 CRC8(poly=0x31, init=0xFF, MSB-first, xorout=0)
  * @param buf 数据, len 数据长度(帧里为 LEN-1)
  * @retval CRC8
  */
uint8_t ProtoLink_CRC8(const uint8_t *buf, uint32_t len);

/* ==================== 初始化 / 接收 ==================== */
void    ProtoLink_Init(proto_link_t *link);

/** 喂 1 字节(可在中断里调)；缓冲满则溢出计数并返回 false(字节被丢) */
bool    ProtoLink_RxPush(proto_link_t *link, uint8_t byte);
/** 喂一段缓冲，返回因缓冲满而丢弃的字节数 */
uint32_t ProtoLink_RxPushBuf(proto_link_t *link, const uint8_t *buf, uint32_t len);

/* ==================== 周期处理 ==================== */
/**
  * @brief 解析接收缓冲：按流式规则找帧、校验、解码并落结构体、刷新计数
  * @param now_ms 单调时钟(ms)，用于记录该通道最近合法帧时刻
  * @retval 本次解析出的合法帧数
  * @note   缓冲不足 2B 或未收齐时等待；LEN 不符/CRC 失败/未知头均丢 1B 重找；
  *         坏帧不刷新 watchdog 时间、不清当前超时位、不计合法帧数。
  */
uint32_t ProtoLink_Process(proto_link_t *link, uint32_t now_ms);

/**
  * @brief 检查三路下行 watchdog 超时并按阈值做故障动作(周期调用)
  * @param now_ms 单调时钟(ms)
  * @note   A1(100ms):vx=vy=w=0；A2(100ms):enable/request=0(保持不回中、停火)；
  *         A3(500ms):两 permit=0。未收到过首帧的通道保持禁用且不计"曾超时"。
  */
void    ProtoLink_Watchdog(proto_link_t *link, uint32_t now_ms);

/* ==================== 取当前生效下行指令(只读) ==================== */
const proto_chassis_cmd_t  *ProtoLink_GetChassisCmd (const proto_link_t *link);
const proto_gimbal_cmd_t   *ProtoLink_GetGimbalCmd  (const proto_link_t *link);
const proto_decision_cmd_t *ProtoLink_GetDecisionCmd(const proto_link_t *link);
uint8_t                     ProtoLink_GetTimeoutFlags(const proto_link_t *link);

/** B3 链路诊断快照：version/flags/三个计数一次取出，保证来自同一逻辑时刻 */
void    ProtoLink_GetLinkStatus(const proto_link_t *link, proto_link_status_t *out);

/* ==================== 上行封包(B1-B4，含 CRC) ==================== */
/** 各 Encode* 从入参快照读全部字段一次后整帧编码，返回整帧长度 */
uint16_t ProtoLink_EncodeGimbalState (uint8_t *out, const proto_gimbal_state_t  *s);
uint16_t ProtoLink_EncodeShootState  (uint8_t *out, const proto_shoot_state_t   *s);
uint16_t ProtoLink_EncodeLinkStatus  (uint8_t *out, const proto_link_status_t   *s);
uint16_t ProtoLink_EncodeChassisState(uint8_t *out, const proto_chassis_state_t *s);

#ifdef __cplusplus
}
#endif

#endif /* __PROTO_LINK_H__ */
