/**
  * @file       proto_link.c
  * @brief      上位机(决策/视觉) <-> 下位机(电控) 自定义通信协议，下位机一侧实现。
  *
  *  协议约束逐条落实：
  *  - LEN 为整帧总长 = payload + 3；
  *  - 逐字节查找已注册头(本侧只收 A1/A2/A3)；不足 2B 等待；
  *  - LEN 与该头不符 -> 丢 1B 重找；
  *  - 长度正确未收齐 -> 等待；收齐先查 CRC；
  *  - CRC 失败 -> 只丢 1B 重找，不按不可信 LEN 跳整帧；
  *  - 成功后消费整帧并继续处理剩余缓冲；
  *  - 接收缓冲有上界并记录溢出；
  *  - CRC/LEN 错误、未知帧、非法 payload 均不刷新 watchdog、不计合法帧数；
  *  - payload 中 float 必须有限，否则拒绝整帧。
  */

/**
 * 仅用于哨兵 使用，请勿用于其他用途！！！
**/
#include "proto_link.h"

/* ==================== 私有常量 ==================== */
#define PROTO_CRC_INIT   0xFFu
#define PROTO_CRC_POLY   0x31u

/* 下行通道编号: A1=0 A2=1 A3=2(与实例数组下标对应) */
#define PROTO_RX_CH_NUM  3u

static const uint8_t rx_head_tab[PROTO_RX_CH_NUM] =
{
    PROTO_HEAD_A1, PROTO_HEAD_A2, PROTO_HEAD_A3
};
static const uint8_t rx_len_tab[PROTO_RX_CH_NUM] =
{
    PROTO_LEN_A1, PROTO_LEN_A2, PROTO_LEN_A3
};
static const uint32_t rx_timeout_tab[PROTO_RX_CH_NUM] =
{
    PROTO_TIMEOUT_A1_MS, PROTO_TIMEOUT_A2_MS, PROTO_TIMEOUT_A3_MS
};
static const uint8_t rx_flag_tab[PROTO_RX_CH_NUM] =
{
    PROTO_FLAG_TIMEOUT_A1, PROTO_FLAG_TIMEOUT_A2, PROTO_FLAG_TIMEOUT_A3
};

/* ==================== CRC8 ==================== */
uint8_t ProtoLink_CRC8(const uint8_t *buf, uint32_t len)
{
    uint8_t crc = PROTO_CRC_INIT;
    uint32_t i;
    for (i = 0; i < len; i++)
    {
        uint8_t byte = buf[i];
        uint32_t j;
        crc ^= byte;
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x80u)
            {
                crc = (uint8_t)((crc << 1) ^ PROTO_CRC_POLY);
            }
            else
            {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ==================== 内部小工具 ==================== */
/* 取 float 的位模式(避免依赖 __STDC_IEC_559__)判断是否有限 */
static bool proto_f32_finite_at(const uint8_t *p)
{
    uint32_t u;
    memcpy(&u, p, 4);
    return (u & 0x7F800000u) != 0x7F800000u;
}

static bool proto_rx_is_head(uint8_t byte, uint8_t *ch)
{
    uint8_t i;
    for (i = 0; i < PROTO_RX_CH_NUM; i++)
    {
        if (rx_head_tab[i] == byte)
        {
            if (ch != 0)
            {
                *ch = i;
            }
            return true;
        }
    }
    return false;
}

static void proto_put_f32(uint8_t *buf, uint32_t off, float v)
{
    memcpy(buf + off, &v, 4);
}

static void proto_put_u16(uint8_t *buf, uint32_t off, uint16_t v)
{
    buf[off]     = (uint8_t)(v & 0xFFu);
    buf[off + 1] = (uint8_t)((v >> 8) & 0xFFu);
}

/* 把整帧(含头/LEN/CRC)追加计算并填入末字节 CRC */
static void proto_finish_frame(uint8_t *buf, uint8_t len)
{
    buf[len - 1] = ProtoLink_CRC8(buf, len - 1);
}

/* ==================== 合法帧落库 / 计数 / watchdog 刷新 ====================
 * 只有"头/长度/CRC/payload"全过的帧才走到这里。
 */
static bool proto_apply_frame(proto_link_t *link, const uint8_t *f, uint32_t now_ms)
{
    proto_chassis_cmd_t  cc;
    proto_gimbal_cmd_t   gc;
    proto_decision_cmd_t dc;
    uint8_t ch;

    switch (f[0])
    {
    case PROTO_HEAD_A1: /* len=15 : vx@2 vy@6 w@10 */
        if (!proto_f32_finite_at(f + 2) ||
            !proto_f32_finite_at(f + 6) ||
            !proto_f32_finite_at(f + 10))
        {
            return false;
        }
        memcpy(&cc.vx, f + 2, 4);
        memcpy(&cc.vy, f + 6, 4);
        memcpy(&cc.w,  f + 10, 4);
        link->chassis_cmd = cc;           /* 一次性提交整帧快照 */
        ch = 0;
        break;

    case PROTO_HEAD_A2: /* len=13 : yaw@2 pitch@6 enable@10 req@11 */
        if (!proto_f32_finite_at(f + 2) ||
            !proto_f32_finite_at(f + 6))
        {
            return false;
        }
        memcpy(&gc.yaw,   f + 2, 4);
        memcpy(&gc.pitch, f + 6, 4);
        gc.gimbal_enable = f[10];
        gc.fire_request  = f[11];
        link->gimbal_cmd = gc;
        ch = 1;
        break;

    case PROTO_HEAD_A3: /* len=5 : gimbal_permit@2 fire_permit@3 */
        dc.gimbal_permit = f[2];
        dc.fire_permit   = f[3];
        link->decision_cmd = dc;
        ch = 2;
        break;

    default:
        return false;
    }

    /* 合法新帧: 仅刷新本通道 watchdog 时间、仅清本通道超时位、计数 */
    link->rx_seen[ch]         = true;
    link->rx_last_ms[ch]      = now_ms;
    link->timeout_flags      &= (uint8_t)~rx_flag_tab[ch];
    link->rx_count[ch]++;               /* uint16 自然回绕 */
    return true;
}

/* ==================== 流式解析(逐字节) ==================== */
/* 本侧只注册下行头 A1/A2/A3；B 帧/未知字节一律逐字节丢弃重找。 */
static uint32_t proto_parse(proto_link_t *link, uint32_t limit, uint32_t now_ms)
{
    uint32_t valid_cnt = 0;
    uint32_t mask = PROTO_RX_RING_SIZE - 1u;

    while (link->tail < limit)
    {
        uint32_t pos;
        uint8_t  hb;
        uint8_t  ch;

        pos = link->tail & mask;
        hb  = link->ring[pos];

        /* 未知头：丢 1B 继续找 */
        if (!proto_rx_is_head(hb, &ch))
        {
            link->tail++;
            link->drop_cnt++;
            continue;
        }

        /* 已见候选头，但不足 2B 取不到 LEN -> 等待 */
        if ((limit - link->tail) < 2u)
        {
            break;
        }

        /* LEN 与本头不符 -> 丢该头 1B 重找 */
        if (link->ring[(link->tail + 1u) & mask] != rx_len_tab[ch])
        {
            link->tail++;
            link->drop_cnt++;
            link->frame_len_err++;
            continue;
        }

        /* 长度正确但未收齐 -> 等待 */
        if ((limit - link->tail) < rx_len_tab[ch])
        {
            break;
        }

        /* 收齐：拷贝出连续帧做校验/解码 */
        {
            uint8_t len = rx_len_tab[ch];
            uint8_t frame[PROTO_MAX_FRAME];
            uint32_t k;
            bool payload_ok;

            for (k = 0; k < len; k++)
            {
                frame[k] = link->ring[(link->tail + k) & mask];
            }

            /* CRC 失败：只丢 1B 重找，不按该 LEN 跳整帧 */
            if (ProtoLink_CRC8(frame, len - 1) != frame[len - 1])
            {
                link->tail++;
                link->drop_cnt++;
                link->frame_crc_err++;
                continue;
            }

            /* 头/长度/CRC 通过：解码落库(payload 非法则拒绝整帧，不刷新不计) */
            payload_ok = proto_apply_frame(link, frame, now_ms);
            if (payload_ok)
            {
                valid_cnt++;
                link->frame_ok++;
            }
            else
            {
                link->frame_payload_err++;
            }

            link->tail += len;          /* 消费整帧 */
        }
    }
    return valid_cnt;
}

/* ==================== 公开 API ==================== */
void ProtoLink_Init(proto_link_t *link)
{
    memset(link, 0, sizeof(*link));
}

bool ProtoLink_RxPush(proto_link_t *link, uint8_t byte)
{
    uint32_t mask = PROTO_RX_RING_SIZE - 1u;
    if ((link->head - link->tail) >= PROTO_RX_RING_SIZE)
    {
        link->overflow_cnt++;           /* 缓冲上界，丢最新字节并记录 */
        return false;
    }
    link->ring[link->head & mask] = byte;
    link->head++;
    return true;
}

uint32_t ProtoLink_RxPushBuf(proto_link_t *link, const uint8_t *buf, uint32_t len)
{
    uint32_t dropped = 0;
    uint32_t i;
    for (i = 0; i < len; i++)
    {
        if (!ProtoLink_RxPush(link, buf[i]))
        {
            dropped++;
        }
    }
    return dropped;
}

uint32_t ProtoLink_Process(proto_link_t *link, uint32_t now_ms)
{
    uint32_t limit = link->head;   /* 本次只消费到进入函数时的水位，避免与喂入竞争 */

    return proto_parse(link, limit, now_ms);
}

void ProtoLink_Watchdog(proto_link_t *link, uint32_t now_ms)
{
    uint8_t ch;
    for (ch = 0; ch < PROTO_RX_CH_NUM; ch++)
    {
        /* 尚未收到过首帧的通道保持禁用，不计"曾超时" */
        if (!link->rx_seen[ch])
        {
            continue;
        }
        if ((now_ms - link->rx_last_ms[ch]) >= rx_timeout_tab[ch])
        {
            link->timeout_flags |= rx_flag_tab[ch];
            switch (ch)
            {
            case 0: /* A1: 停车停转 */
                link->chassis_cmd.vx = 0.0f;
                link->chassis_cmd.vy = 0.0f;
                link->chassis_cmd.w  = 0.0f;
                break;
            case 1: /* A2: enable/request 归 0，保持 yaw/pitch 不回中 */
                link->gimbal_cmd.gimbal_enable = 0u;
                link->gimbal_cmd.fire_request  = 0u;
                break;
            case 2: /* A3: 两 permit 归 0 */
                link->decision_cmd.gimbal_permit = 0u;
                link->decision_cmd.fire_permit   = 0u;
                break;
            default:
                break;
            }
        }
    }
}

const proto_chassis_cmd_t  *ProtoLink_GetChassisCmd (const proto_link_t *link)
{
    return &link->chassis_cmd;
}

const proto_gimbal_cmd_t   *ProtoLink_GetGimbalCmd  (const proto_link_t *link)
{
    return &link->gimbal_cmd;
}

const proto_decision_cmd_t *ProtoLink_GetDecisionCmd(const proto_link_t *link)
{
    return &link->decision_cmd;
}

uint8_t ProtoLink_GetTimeoutFlags(const proto_link_t *link)
{
    return link->timeout_flags;
}

/* 三个计数与 flags 一次拷出，保证 B3 编码时来自同一份链路诊断快照 */
void ProtoLink_GetLinkStatus(const proto_link_t *link, proto_link_status_t *out)
{
    out->proto_version = PROTO_VERSION;
    out->timeout_flags = link->timeout_flags;
    out->rx_count_a1   = link->rx_count[0];
    out->rx_count_a2   = link->rx_count[1];
    out->rx_count_a3   = link->rx_count[2];
}

/* ==================== 上行封包 ==================== */
/* B1 GimbalState 27B : B1 | 1B | q(wxyz) | yaw | pitch | CRC */
uint16_t ProtoLink_EncodeGimbalState(uint8_t *out, const proto_gimbal_state_t *s)
{
    out[0] = PROTO_HEAD_B1;
    out[1] = PROTO_LEN_B1;
    proto_put_f32(out, 2,  s->q[0]);
    proto_put_f32(out, 6,  s->q[1]);
    proto_put_f32(out, 10, s->q[2]);
    proto_put_f32(out, 14, s->q[3]);
    proto_put_f32(out, 18, s->yaw);
    proto_put_f32(out, 22, s->pitch);
    proto_finish_frame(out, PROTO_LEN_B1);
    return PROTO_LEN_B1;
}

/* B2 ShootState 9B : B2 | 1B | bullet_speed | bullet_count(u16) | CRC */
uint16_t ProtoLink_EncodeShootState(uint8_t *out, const proto_shoot_state_t *s)
{
    out[0] = PROTO_HEAD_B2;
    out[1] = PROTO_LEN_B2;
    proto_put_f32(out, 2, s->bullet_speed);
    proto_put_u16(out, 6, s->bullet_count);
    proto_finish_frame(out, PROTO_LEN_B2);
    return PROTO_LEN_B2;
}

/* B3 LinkStatus 11B : B3 | 1B | ver | flags | rx_a1(u16) | rx_a2 | rx_a3 | CRC */
uint16_t ProtoLink_EncodeLinkStatus(uint8_t *out, const proto_link_status_t *s)
{
    out[0] = PROTO_HEAD_B3;
    out[1] = PROTO_LEN_B3;
    out[2] = s->proto_version;
    out[3] = s->timeout_flags;
    proto_put_u16(out, 4, s->rx_count_a1);
    proto_put_u16(out, 6, s->rx_count_a2);
    proto_put_u16(out, 8, s->rx_count_a3);
    proto_finish_frame(out, PROTO_LEN_B3);
    return PROTO_LEN_B3;
}

/* B4 ChassisState 20B : B4 | 1B | vx | vy | w | yaw_odom | flags | CRC */
uint16_t ProtoLink_EncodeChassisState(uint8_t *out, const proto_chassis_state_t *s)
{
    out[0] = PROTO_HEAD_B4;
    out[1] = PROTO_LEN_B4;
    proto_put_f32(out, 2,  s->vx);
    proto_put_f32(out, 6,  s->vy);
    proto_put_f32(out, 10, s->w);
    proto_put_f32(out, 14, s->yaw_odom);
    out[18] = s->flags;
    proto_finish_frame(out, PROTO_LEN_B4);
    return PROTO_LEN_B4;
}
