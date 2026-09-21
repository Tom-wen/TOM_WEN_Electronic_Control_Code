#include "data_processing.h"
#include "gimbal.h"
#include "uav_gimbal.h"
#include "shoot.h"
#include "Circular_buffer.h"
#include "control_power.h"
#include "Lowpass.h"
#include "usbd_cdc_if.h"
#include "proto_link.h"

/* 上位机上行(B1-B4)总开关：1=周期组帧 usb_send，0=停发(可在 config.h 提前定义覆盖) */
#ifndef HOST_REPORT_TX_ENABLE
#define HOST_REPORT_TX_ENABLE 1
#endif

extern QueueHandle_t rs485_queueHandle;
extern float pitch_angle;
board_data received_data;
//双板发送数据
board_data gimbal_board_data;
GimbalToVision Gimbal_Vision;
//上位机上行聚合快照(填好字段后由 HostReport_Send 拆分发送)
host_report_t host_report;//哨兵给上位机发送的数据
uint16_t motor_angle[6];
uint8_t sent_flag = 1;
uint8_t sent_data[10] ={0xA5, 0x01, 0x00, 0x00, 0x68, 0x01, 0x0F, 0x01, 0x8B, 0xBD};//旧图传设置
float target_speed = 0;
float target_angle = 0;
float feedback_angle = 0;
float current = 0;
float voltage = 0;
float power = 0;
float speed = 0;
float feedback_current = 0;
float measure_power = 0;
float pid_out = 0;
float pid_error = 0;
float pid_lasterror = 0;

CDC_RxStats_t cdc_stats = {0};
SBUS_RxStats_t sbus_stats = {0};
uint16_t count=0;
uint32_t last_command_seq=0;
uint32_t last_delta=0;
uint32_t delta_max=1;



void data_task(void *argument)
{
    UART_RX_TypeDef *pRecvUartData; /* 定义指向串口数据的指针 */
    UART_RX_TypeDef *pRecvUartData1; /* 定义指向串口数据的指针 */
    //Upper_Computer_Init(&target_angle);    
    //Upper_Computer_Init(&feedback_angle); 

    //Upper_Computer_Init(&Gimbal_6020[0].motor_data->pid[1].Out);    
    // Upper_Computer_Init(&pid_out);
    //Upper_Computer_Init(&INS.Yaw);
    // Upper_Computer_Init(&pid_error);
    // Upper_Computer_Init(&pid_lasterror);
    // Upper_Computer_Init(&INS.Gyro[Zt]);
    //Upper_Computer_Init(&INS.Pitch);
    // Upper_Computer_Init(&INS.Roll);
    // Upper_Computer_Init(&INS.temp);
    // Upper_Computer_Init(&chassis_3508_motor[0].cal_current);
    // Upper_Computer_Init(&chassis_3508_motor[0].decay_number);
    // Upper_Computer_Init(&motor3508_current_power[0]);
    //Upper_Computer_Init(&Chassis_3508[0].motor_data->pid[1].Out);
    // Upper_Computer_Init(&voltage);
    // Upper_Computer_Init(&current);
    // Upper_Computer_Init(&power);
    //Upper_Computer_Init(&speed);
    // Upper_Computer_Init(&feedback_current);
    // Upper_Computer_Init(&power_receive_data.pm_current);
    // Upper_Computer_Init(&power_receive_data.pm_power);
    // Upper_Computer_Init(&Chassis_3508[0].motor_data->feedback->current);    
    // Upper_Computer_Init(&Chassis_3508[0].motor_data->feedback->vel);    
    // Upper_Computer_Init(&Shoot_3508[0].motor_data->feedback->vel); 
    /* Infinite loop */
    for (;;)
    {
        // count++;
        // if(test_feedback.command_seq-last_command_seq >last_delta )
        // {
        //     delta_max = test_feedback.command_seq-last_command_seq;
        // }

        // last_delta =test_feedback.command_seq-last_command_seq;
        // last_command_seq=test_feedback.command_seq;
        //target_speed = Shoot_3508[0].motor_data->target_velocity;
        /* 一直等待直到有数据 */
        /* 参数1：队列handle */
        /* 参数2：放入接收消息的指针，也就是让它指向串口中断中已接收串口数据完成的结构体 */
        #ifdef COMPILE_ARM
        uint8_t cmd[14];
        if (Angle_GetCommand(cmd))
        {
            for (int i = 0; i < 6; i++) 
            {
                motor_angle[i] = (cmd[i * 2 + 1] << 8) | cmd[i * 2 + 2];
            }
        }

        #endif
        if(xQueueReceive(rs485_queueHandle, &pRecvUartData1, 0) == pdTRUE)
        {
            if(pRecvUartData1->size == sizeof(board_data))
            {
                if(verify_CRC16_check_sum((uint8_t*)pRecvUartData1->buffer, sizeof(board_data)))
                {
                    memcpy(&received_data, pRecvUartData1->buffer, pRecvUartData1->size);
                }
            }
        }
        //双板通信任务
        #ifdef gimbal_board
            Board_data_Init(&gimbal_board_data);
            usart_tx_dma_send(&huart3, (uint8_t*)&gimbal_board_data, sizeof(gimbal_board_data));
        #endif
        //图传发送标志位
        // if(sent_flag != 2)
        // {
        //     usart_tx_dma_send(&huart10, sent_data, sizeof(sent_data));
        //     sent_flag = 2;
        // }
        //视觉发送：收到上位机一包，回发一包
        #if defined(COMPILE_GIMBAL) || defined(COMPILE_UAV_GIMBAL) || defined(ROBOTIC_SWING_GIMBAL)
        {
            // static uint32_t last_rx_vision = 0;
            // if(cdc_rx_vision != last_rx_vision)
            // {
            //     last_rx_vision = cdc_rx_vision;
                Gimbal_sent_vision(&Gimbal_Vision, &INS);
                usb_send((uint8_t*)&Gimbal_Vision, sizeof(Gimbal_Vision));
            // }
            // test_sent_vision(&test,&INS);//自瞄的丢包率检测
            // usb_send((uint8_t*)&test, sizeof(test));
        }
        #endif
        #if defined AUTO_SENTRY
        {
        // 新协议 A1/A2/A3：周期解析 USB CDC 接收缓冲 + 三路 watchdog
        // B1-B4：向上位机上报(聚合快照拆帧发送)
        {
            uint32_t proto_now = xTaskGetTickCount();
            ProtoLink_Process(&cdc_proto_link, proto_now);
            ProtoLink_Watchdog(&cdc_proto_link, proto_now);
            HostReport_Send(proto_now);
        }
        }
        #endif
        //target_angle = Trigger_3508[0].motor_data->target_position;
        //feedback_angle = Trigger_3508[0].motor_data->total_angle;
        // voltage = power_data.voltage;
        // current = power_data.current;
        // power = power_data.power;
        //speed = Shoot_3508[0].motor_data->feedback->vel;
        //feedback_current = fabs(Chassis_3508[0].motor_data->pid[1].Out);
        //measure_power = motor3508_current_power[0];
        // pid_out = Gimbal_6020[0].motor_data->pid[2].Out;
        // pid_error = Gimbal_6020[0].motor_data->pid[2].error;
        // pid_lasterror = Gimbal_6020[0].motor_data->pid[2].last_reference;
        //key_edge_update(); 
        //usart_vofa_send(&huart1);

        // 每 500ms 更新一次 CDC 接收包统计（2ms 循环 / 250 次）
        static uint16_t stats_cnt = 0;
        if(++stats_cnt >= 250)
        {
            stats_cnt = 0;
            CDC_RxStats_Get(&cdc_stats);
            SBUS_RxStats_Get(&sbus_stats);
        }

        vTaskDelay(pdMS_TO_TICKS(2));        
    }
}

#ifdef gimbal_board
//双板初始化
void Board_data_Init(board_data *sent_data)
{
    #ifdef COMPILE_HERO_GIMBAL
    sent_data->sbus_online = sbus_online;
    sent_data->yaw_position = Gimbal_4310[0].motor_data->feedback->pos;
    sent_data->yaw_speed = Gimbal_4310[0].motor_data->feedback->vel;
    sent_data->ins_yaw = INS.Yaw;
    memcpy(&sent_data->rc_ctrl, &rc_ctrl, sizeof(RC_ctrl_t));
    append_CRC16_check_sum((uint8_t*)sent_data, sizeof(board_data));
    #endif
    #ifdef COMPILE_ARM
    sent_data->sbus_online = sbus_online;
    sent_data->ins_yaw = INS.Yaw;
    memcpy(&sent_data->rc_ctrl, &rc_ctrl, sizeof(RC_ctrl_t));
    append_CRC16_check_sum((uint8_t*)sent_data, sizeof(board_data));
    #endif    
}
#endif

#if defined(COMPILE_GIMBAL) || defined(COMPILE_UAV_GIMBAL) || defined(COMPILE_HERO_GIMBAL) || defined(ROBOTIC_SWING_GIMBAL)
//云台发送视觉数据
void Gimbal_sent_vision(GimbalToVision *sent_vision, INS_t *ins)
{
    sent_vision->head[0] = 'G';
    sent_vision->head[1] = 'V';
    sent_vision->mode = 0;
    sent_vision->q[0] = ins->q[0];
    sent_vision->q[1] = ins->q[1];
    sent_vision->q[2] = ins->q[2];
    sent_vision->q[3] = ins->q[3];
    sent_vision->yaw = ins->Yaw / 57.29578;
    //sent_vision->yaw = yaw_angle / 57.29578;
    sent_vision->yaw_vel = INS.Gyro[Zt];
    //sent_vision->pitch = pitch_angle / 57.29578;
    sent_vision->pitch = ins->Pitch / 57.29578;
    sent_vision->pitch_vel = INS.Gyro[Zt];
    #ifdef COMPILE_REFEREE
    sent_vision->bullet_speed = referee_data.shoot_data.initial_speed;
    #endif
    sent_vision->bullet_count = 0;
    sent_vision->tail = 'G';
}

#endif


/**
 * @brief  CDC 接收包统计检测
 * @param  stats  输出统计结果的结构体指针
 * @note   在固定周期任务里调用（建议 1000ms），可通过 Ozone/vofa 观察各计数器
 *         total   : 所有触发 CDC_Receive_HS 的包（含格式错误包）
 *         vision  : 帧头帧尾匹配的 VisionToGimbal 有效包
 *         auto_fb : 帧头帧尾匹配的 auto_feedback 有效包
 *         rate_hz : 上一个统计周期内 vision 包的实际到达频率（Hz）
 */
void CDC_RxStats_Get(CDC_RxStats_t *stats)
{
    static uint32_t last_vision = 0;
    static uint32_t last_tick   = 0;

    uint32_t now     = xTaskGetTickCount();
    uint32_t elapsed = now - last_tick;

    stats->total    = cdc_rx_total;
    stats->vision   = cdc_rx_vision;
    stats->auto_fb  = cdc_rx_auto;

    if(elapsed > 0)
        stats->rate_hz = (float)(cdc_rx_vision - last_vision) * 1000.0f / (float)elapsed;
    else
        stats->rate_hz = 0.0f;

    last_vision = cdc_rx_vision;
    last_tick   = now;
}

/**
 * @brief  UART5 遥控器接收包统计检测
 * @param  stats  输出统计结果的结构体指针
 * @note   每 500ms 由 data_task 调用一次，通过 Ozone Watch 观察 sbus_stats
 *         total   : UART5 所有触发回调的包数（含丢帧包）
 *         valid   : sbus_online == 1 时的有效包数
 *         rate_hz : 上一统计周期内有效包的实际频率（Hz），正常应约为 71 Hz
 */
void SBUS_RxStats_Get(SBUS_RxStats_t *stats)
{
    static uint32_t last_valid = 0;
    static uint32_t last_tick  = 0;

    uint32_t now     = xTaskGetTickCount();
    uint32_t elapsed = now - last_tick;

    stats->total   = sbus_rx_total;
    stats->valid   = sbus_rx_valid;

    if(elapsed > 0)
        stats->rate_hz = (float)(sbus_rx_valid - last_valid) * 1000.0f / (float)elapsed;
    else
        stats->rate_hz = 0.0f;

    last_valid = sbus_rx_valid;
    last_tick  = now;
}

/* =======================================================================
 * 上位机上行(B1-B4)：先清零再按车型补真实数据。
 * 目前仅留宏开关+空实现，请按你的车型在对应 #ifdef 分支里填写来源。
 * 例(哨兵参考符号，非现成可编译)：
 *   r->gimbal_q[i]        = INS.q[i];
 *   r->gimbal_yaw         = 云台当前绝对角(rad,左正);
 *   r->gimbal_pitch       = 云台当前俯仰(rad,抬头为负);
 *   r->chassis_vx/vy/w    = 底盘实测(同一采样);
 *   r->yaw_odom           = 云台相对底盘编码器角(与速度同刻);
 *   r->chassis_flags      = 功率限幅标志;
 *   r->bullet_speed       = 弹速;  r->bullet_count = 累计发射;
 * ===================================================================== */
void HostReport_Collect(host_report_t *r)
{
    /* 先清零：未被本周期填充的字段不发上一次旧值 */
    memset(r, 0, sizeof(*r));

#if defined(AUTO_SENTRY) || defined(COMPILE_HERO_CHASSIS) || defined(COMPILE_AGV_CHASSIS)
    /* TODO: 按你车型在此填上行字段 */
    /* r->gimbal_q[0] = ...;  r->gimbal_yaw = ...;  r->gimbal_pitch = ...; */
    /* r->chassis_vx = ...;   r->chassis_vy = ...;  r->chassis_w = ...;   */
    /* r->yaw_odom = ...;     r->chassis_flags = ...;                      */
    /* r->bullet_speed = ...; r->bullet_count = ...;                       */
#else
    /* 其它车型分支 */
    (void)r;
#endif
}

void HostReport_Send(uint32_t now_ms)
{
#if HOST_REPORT_TX_ENABLE
    uint8_t tx[PROTO_MAX_FRAME];
    static uint32_t last_b1 = 0;
    static uint32_t last_b2 = 0;
    static uint32_t last_b3 = 0;
    static uint32_t last_b4 = 0;

    /* host_report 由你在本任务外/本周期前填好，这里直接快照组帧发送。
     * 注意：不再调用 HostReport_Collect()，以免其 memset 清零覆盖你填的数据。
     * 帧内一致由你保证：同一帧的字段请在同一逻辑周期内更新一次。 */

    /* B1 GimbalState 200Hz(5ms) */
    if((now_ms - last_b1) >= 5u)
    {
        proto_gimbal_state_t gs;
        uint8_t i;
        for(i = 0; i < 4; i++)
        {
            gs.q[i] = host_report.gimbal_q[i];
        }
        gs.yaw   = host_report.gimbal_yaw;
        gs.pitch = host_report.gimbal_pitch;
        usb_send(tx, ProtoLink_EncodeGimbalState(tx, &gs));
        last_b1 = now_ms;
    }

    /* B4 ChassisState 40Hz(25ms)，协议允许 20-50Hz */
    if((now_ms - last_b4) >= 25u)
    {
        proto_chassis_state_t cs;
        cs.vx       = host_report.chassis_vx;
        cs.vy       = host_report.chassis_vy;
        cs.w        = host_report.chassis_w;
        cs.yaw_odom = host_report.yaw_odom;
        cs.flags    = host_report.chassis_flags;
        usb_send(tx, ProtoLink_EncodeChassisState(tx, &cs));
        last_b4 = now_ms;
    }

    /* B2 ShootState 20Hz(50ms) */
    if((now_ms - last_b2) >= 50u)
    {
        proto_shoot_state_t ss;
        ss.bullet_speed = host_report.bullet_speed;
        ss.bullet_count = host_report.bullet_count;
        usb_send(tx, ProtoLink_EncodeShootState(tx, &ss));
        last_b2 = now_ms;
    }

    /* B3 LinkStatus 10Hz(100ms)：版本/超时标志/计数取自链路快照 */
    if((now_ms - last_b3) >= 100u)
    {
        proto_link_status_t ls;
        ProtoLink_GetLinkStatus(&cdc_proto_link, &ls);
        usb_send(tx, ProtoLink_EncodeLinkStatus(tx, &ls));
        last_b3 = now_ms;
    }
#else
    (void)now_ms;
#endif
}
