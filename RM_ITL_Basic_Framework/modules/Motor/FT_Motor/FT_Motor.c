#include "FT_Motor.h"
#include "bsp_usart.h"

/* ===================== 内部变量 ===================== */

static UART_HandleTypeDef *ft_huart;
static uint8_t ft_tx_buf[FT_TX_BUF_SIZE];
static uint16_t ft_tx_len;

FT_Feedback ft_feedback[MAX_FT_SERVOS];

/* ===================== 内部函数 ===================== */

static void ft_tx_reset(void)
{
    ft_tx_len = 0;
}

static void ft_tx_byte(uint8_t b)
{
    if (ft_tx_len < FT_TX_BUF_SIZE)
        ft_tx_buf[ft_tx_len++] = b;
}

/* 校验和: ~(buf[0]+buf[1]+...+buf[len-1]) & 0xFF */
static uint8_t ft_checksum(const uint8_t *buf, uint16_t len)
{
    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++)
        sum += buf[i];
    return (~sum) & 0xFF;
}

/* 构建并立即发送一个 WRITE 指令包 */
static void ft_send_write(uint8_t id, uint8_t mem_addr, const uint8_t *data, uint8_t data_len)
{
    ft_tx_reset();
    ft_tx_byte(0xFF);
    ft_tx_byte(0xFF);
    ft_tx_byte(id);
    ft_tx_byte(data_len + 2 + 1); /* data_len + MemAddr(1) + Inst(1) + 1 */
    ft_tx_byte(FT_INST_WRITE);
    ft_tx_byte(mem_addr);
    for (uint8_t i = 0; i < data_len; i++)
        ft_tx_byte(data[i]);
    ft_tx_byte(ft_checksum(&ft_tx_buf[2], ft_tx_len - 2));

    usart_tx_dma_send(ft_huart, ft_tx_buf, ft_tx_len);
}

/* ===================== 初始化 ===================== */

void FT_Motor_Init(UART_HandleTypeDef *huart)
{
    ft_huart = huart;
    ft_tx_len = 0;
    for (uint8_t i = 0; i < MAX_FT_SERVOS; i++) {
        ft_feedback[i].online = 0;
        ft_feedback[i].position = 0;
        ft_feedback[i].speed = 0;
        ft_feedback[i].load = 0;
        ft_feedback[i].current = 0;
    }
    bsp_usart8_set_callback(FT_ParseResponse);
}

void FT_Process(void (*control_fn)(void *), void *arg)
{
    static uint8_t phase = 0;

    if (phase == 0) {
        if (control_fn) control_fn(arg);
    } else {
        FT_PollFeedback();
    }
    phase = !phase;
}
/* ===================== 单个舵机位置控制 ===================== */

void FT_WritePos(uint8_t id, int16_t pos, uint16_t speed, uint8_t acc)
{
    uint16_t pos_val = (pos < 0) ? ((uint16_t)(-pos) | 0x8000) : (uint16_t)pos;

    uint8_t data[7];
    data[0] = acc;
    data[1] = pos_val & 0xFF;        /* Position L */
    data[2] = (pos_val >> 8) & 0xFF;  /* Position H */
    data[3] = 0;                       /* Time L */
    data[4] = 0;                       /* Time H */
    data[5] = speed & 0xFF;           /* Speed L */
    data[6] = (speed >> 8) & 0xFF;    /* Speed H */

    ft_send_write(id, FT_REG_ACC, data, 7);
}

// 多个舵机同步位置、速度、加速度控制
void FT_SyncWritePos(const uint8_t ids[], uint8_t count,
                     const int16_t pos[], const uint16_t speed[], const uint8_t acc[])
{
    uint8_t data_len = 7; /* acc(1) + pos(2) + time(2) + speed(2) */
    uint8_t length = (data_len + 1) * count + 4;

    ft_tx_reset();
    ft_tx_byte(0xFF);
    ft_tx_byte(0xFF);
    ft_tx_byte(FT_BROADCAST_ID);
    ft_tx_byte(length);
    ft_tx_byte(FT_INST_SYNC_WRITE);
    ft_tx_byte(FT_REG_ACC);   /* 起始地址 */
    ft_tx_byte(data_len);     /* 每个舵机写入的数据长度 */

    for (uint8_t i = 0; i < count; i++) {
        uint16_t pos_val = (pos[i] < 0) ? ((uint16_t)(-pos[i]) | 0x8000) : (uint16_t)pos[i];
        uint16_t spd = speed ? speed[i] : 0;

        ft_tx_byte(ids[i]);
        ft_tx_byte(acc ? acc[i] : 0);
        ft_tx_byte(pos_val & 0xFF);
        ft_tx_byte((pos_val >> 8) & 0xFF);
        ft_tx_byte(0);  /* Time L */
        ft_tx_byte(0);  /* Time H */
        ft_tx_byte(spd & 0xFF);
        ft_tx_byte((spd >> 8) & 0xFF);
    }

    ft_tx_byte(ft_checksum(&ft_tx_buf[2], ft_tx_len - 2));
    usart_tx_dma_send(ft_huart, ft_tx_buf, ft_tx_len);
}

/* ===================== 力位混合控制 ===================== */

void FT_WritePosTorque(uint8_t id, int16_t pos, uint16_t torque, uint16_t speed, uint8_t acc)
{
    uint16_t pos_val = (pos < 0) ? ((uint16_t)(-pos) | 0x8000) : (uint16_t)pos;

    uint8_t data[8];
    data[0] = 1;                        /* Enable = 1 开启力矩 */
    data[1] = acc;
    data[2] = pos_val & 0xFF;           /* Position L */
    data[3] = (pos_val >> 8) & 0xFF;    /* Position H */
    data[4] = torque & 0xFF;            /* Torque L */
    data[5] = (torque >> 8) & 0xFF;     /* Torque H */
    data[6] = speed & 0xFF;             /* Speed L */
    data[7] = (speed >> 8) & 0xFF;      /* Speed H */

    ft_send_write(id, FT_REG_TORQUE_ENABLE, data, 8);
}

void FT_SyncWritePosTorque(const uint8_t ids[], uint8_t count,
                           const int16_t pos[], const uint16_t torque[],
                           const uint16_t speed[], const uint8_t acc[])
{
    uint8_t data_len = 8; /* enable(1)+acc(1)+pos(2)+torque(2)+speed(2) */
    uint8_t length = (data_len + 1) * count + 4;

    ft_tx_reset();
    ft_tx_byte(0xFF);
    ft_tx_byte(0xFF);
    ft_tx_byte(FT_BROADCAST_ID);
    ft_tx_byte(length);
    ft_tx_byte(FT_INST_SYNC_WRITE);
    ft_tx_byte(FT_REG_TORQUE_ENABLE);
    ft_tx_byte(data_len);

    for (uint8_t i = 0; i < count; i++) {
        uint16_t pos_val = (pos[i] < 0) ? ((uint16_t)(-pos[i]) | 0x8000) : (uint16_t)pos[i];
        uint16_t trq = torque ? torque[i] : 0;
        uint16_t spd = speed ? speed[i] : 0;

        ft_tx_byte(ids[i]);
        ft_tx_byte(1);                          /* Enable */
        ft_tx_byte(acc ? acc[i] : 0);
        ft_tx_byte(pos_val & 0xFF);
        ft_tx_byte((pos_val >> 8) & 0xFF);
        ft_tx_byte(trq & 0xFF);
        ft_tx_byte((trq >> 8) & 0xFF);
        ft_tx_byte(spd & 0xFF);
        ft_tx_byte((spd >> 8) & 0xFF);
    }

    ft_tx_byte(ft_checksum(&ft_tx_buf[2], ft_tx_len - 2));
    usart_tx_dma_send(ft_huart, ft_tx_buf, ft_tx_len);
}

/* ===================== 速度控制 ===================== */

void FT_WriteSpeed(uint8_t id, int16_t speed, uint8_t acc)
{
    uint16_t spd_val = (speed < 0) ? ((uint16_t)(-speed) | 0x8000) : (uint16_t)speed;

    uint8_t data[7];
    data[0] = acc;
    data[1] = 0;  /* Position L */
    data[2] = 0;  /* Position H */
    data[3] = 0;  /* Time L */
    data[4] = 0;  /* Time H */
    data[5] = spd_val & 0xFF;
    data[6] = (spd_val >> 8) & 0xFF;

    ft_send_write(id, FT_REG_ACC, data, 7);
}

/* ===================== 模式 & 力矩 ===================== */

void FT_SetMode(uint8_t id, uint8_t mode)
{
    ft_send_write(id, FT_REG_MODE, &mode, 1);
}

void FT_TorqueEnable(uint8_t id, uint8_t enable)
{
    ft_send_write(id, FT_REG_TORQUE_ENABLE, &enable, 1);
}

/* ===================== 反馈 ===================== */

void FT_ReadFeedback(uint8_t id)
{
    ft_tx_reset();
    ft_tx_byte(0xFF);
    ft_tx_byte(0xFF);
    ft_tx_byte(id);
    ft_tx_byte(4);                   /* Length = 2(params) + 2 */
    ft_tx_byte(FT_INST_READ);
    ft_tx_byte(FT_REG_PRESENT_POS_L); /* 起始地址 */
    ft_tx_byte(FT_FEEDBACK_DATA_LEN);  /* 读取长度 */
    ft_tx_byte(ft_checksum(&ft_tx_buf[2], ft_tx_len - 2));

    usart_tx_dma_send(ft_huart, ft_tx_buf, ft_tx_len);
}

//多舵机状态轮询查询，防止出现丢包情况
void FT_PollFeedback(void)
{
    static uint8_t poll_id = 1;  // 从ID=1开始
    FT_ReadFeedback(poll_id);
    poll_id++;
    if (poll_id > 6)  // 你的舵机总数
        poll_id = 1;
}

/*
 * 解析舵机返回帧, 在 UART 空闲中断回调里调用
 *
 * 帧格式: FF FF ID Length ERROR Data[0..n-1] CheckSum
 * 其中 Length = 数据字节数 + 2 (ERROR + CheckSum占的额外字节)
 */
void FT_ParseResponse(uint8_t *data, uint16_t len)
{
    /* 最小帧: FF FF ID Length ERROR CheckSum = 6字节 */
    if (len < 6)
        return;

    /* 找帧头 FF FF */
    uint16_t idx = 0;
    while (idx + 5 < len) {
        if (data[idx] == 0xFF && data[idx + 1] == 0xFF && data[idx + 2] != 0xFF)
            break;
        idx++;
    }
    if (idx + 5 >= len)
        return;

    uint8_t id     = data[idx + 2];
    uint8_t length = data[idx + 3];

    /* 检查帧完整性 */
    if (idx + length + 4 > len)
        return;

    /* 校验和: 覆盖 ID ~ 最后一个数据字节 */
    uint8_t ck = ft_checksum(&data[idx + 2], length + 1);
    if (ck != data[idx + length + 3])
        return;

    /* 数据字节数 = Length - 2 */
    uint8_t data_len = length - 2;
    if (data_len == 0)
        return;  /* 仅ACK, 无数据 */

    const uint8_t *payload = &data[idx + 5]; /* Data起始 */

    /* 解析反馈数据 (从地址56读取的15字节) */
    if (id >= MAX_FT_SERVOS)
        return;

    FT_Feedback *fb = &ft_feedback[id-1];

    if (data_len >= 8) {
        /* Position: bit15=方向, bits0-14=值 */
        uint16_t raw_pos = payload[0] | (payload[1] << 8);
        fb->position = (raw_pos & 0x8000) ? -(raw_pos & 0x7FFF) : (raw_pos & 0x7FFF);

        /* Speed: bit15=方向, bits0-14=值, 单位: 0.732RPM */
        uint16_t raw_spd = payload[2] | (payload[3] << 8);
        fb->speed = (raw_spd & 0x8000) ? -(raw_spd & 0x7FFF) : (raw_spd & 0x7FFF);

        /* Load: bit10=方向, bits0-9=值, 单位: 0.1% */
        uint16_t raw_load = payload[4] | (payload[5] << 8);
        fb->load = (raw_load & 0x400) ? -(raw_load & 0x3FF) : (raw_load & 0x3FF);

        fb->voltage     = payload[6];
        fb->temperature = payload[7];
    }

    if (data_len >= 15) {
        fb->moving  = payload[10];
        /* Current: 无方向位, 单位: 6.5mA */
        uint16_t raw_cur = payload[13] | (payload[14] << 8);
        fb->current = (int16_t)raw_cur;
    }

    fb->online = 1;
}
