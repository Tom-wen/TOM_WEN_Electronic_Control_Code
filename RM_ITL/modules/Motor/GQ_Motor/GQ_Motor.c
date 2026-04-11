#include "GQ_Motor.h"

//GQ电机接收数据数组，支持多个电机
Motor_feedback GQ_Motor_RX[MAX_CAN][MAX_GQ_MOTORS];
// 电机ID映射表（将CAN ID映射到电机数组索引）
uint16_t gq_motor_id_map[MAX_GQ_MOTORS] = {256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266};

//高擎电机初始化
void GQ_Motor_Init(MotorInstance *motors)
{
    for(int i = 0; i < motors[0].motor_count; i ++)
    {
        timed_return_motor_status(motors[i].motor_data->hfdcan, motors[i].motor_data->id, 10);
        DWT_Delay(0.01);
    }
}

/**
 * @brief 速度控制
 * @param id 电机ID
 * @param vel 速度：单位 0.00025 转/秒，如 val = 1000 表示 0.25 转/秒
 * @param tqe 力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM
 */
void GQ_Motor_Speed_mode(MotorInstance *motors)
{
    uint8_t data[8] = {0x07, 0x07, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00};				//发送数据
    for(int i = 0; i < motors[0].motor_count; i ++)
    {
        if(motors[i].motor_data->motor_enable != 0)
        {
            *(int16_t *)&data[4] = motors[i].motor_data->target_velocity;
            *(int16_t *)&data[6] = motors[i].motor_data->target_current;
            fdcanx_send_data(motors[i].motor_data->hfdcan, (0x8000 | motors[i].motor_data->id), data, 8);
        }
        else 
        {      
            *(int16_t *)&data[4] = 0;
            *(int16_t *)&data[6] = 0;
            fdcanx_send_data(motors[i].motor_data->hfdcan, (0x8000 | motors[i].motor_data->id), data, 8);
        }
        DWT_Delay(0.007);
    }
}

/**
 * @brief 力矩模式
 * @param id 电机ID
 * @param tqe 力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM
 */
void GQ_Motor_Tqe_mode(MotorInstance *motors)
{
    uint8_t data[8] = {0x05, 0x13, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00};				//发送数据
    for(int i = 0; i < motors[0].motor_count; i ++)
    {
        if(motors[i].motor_data->motor_enable != 0)
        {
            *(int16_t *)&data[2] = motors[i].motor_data->target_current;
            fdcanx_send_data(motors[i].motor_data->hfdcan, (0x8000 | motors[i].motor_data->id), data, 8);
        }
        else 
        {      
            *(int16_t *)&data[2] = 0;
            fdcanx_send_data(motors[i].motor_data->hfdcan, (0x8000 | motors[i].motor_data->id), data, 8);
        }
        DWT_Delay(0.007);
    }
}

/**
 * @brief 电机位置-速度-最大力矩控制，int16型
 * @param id  电机ID
 * @param pos 位置：单位 0.0001 圈，如 pos = 5000 表示转到 0.5 圈的位置。
 * @param vel 速度：单位 0.00025 转/秒，如 val = 1000 表示 0.25 转/秒
 * @param tqe 最大力矩：单位：0.01 NM，如 torque = 110 表示最大力矩为 1.1NM
 */
void GQ_Motor_Pos_Vel_mode(MotorInstance *motors)
{
    uint8_t data[8] = {0x07, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};				//发送数据
    for(int i = 0; i < motors[0].motor_count; i ++)
    {
        if(motors[i].motor_data->motor_enable != 0)
        {
            *(int16_t *)&data[2] = motors[i].motor_data->target_velocity;
            *(int16_t *)&data[4] = motors[i].motor_data->target_current;
            *(int16_t *)&data[6] = motors[i].motor_data->target_position;
            fdcanx_send_data(motors[i].motor_data->hfdcan, (0x8000 | motors[i].motor_data->id), data, 8);
        }
        else 
        {      
            *(int16_t *)&data[2] = 0;
            *(int16_t *)&data[4] = 0;
            *(int16_t *)&data[6] = 0;
            fdcanx_send_data(motors[i].motor_data->hfdcan, (0x8000 | motors[i].motor_data->id), data, 8);
        }
        DWT_Delay(0.007);
    }
}

//周期返回电机状态数据
//周期单位为ms，最小周期为1ms
void timed_return_motor_status(FDCAN_HandleTypeDef *hfdcan, uint8_t id, int16_t t_ms)
{
    uint8_t tdata[] = {0x05, 0xb4, 0x02, 0x00, 0x00};
    *(int16_t *)&tdata[3] = t_ms;
    fdcanx_send_data(hfdcan, 0x8000 | id, tdata, sizeof(tdata));
}

//高擎电机反馈数据解析
void GQ_motor_can_callback(CANRxData *Rx_data, CAN_PORT can_port)
{
    // 边界检查
    if(can_port > CAN3)
    {
      return;
    }
    // 查找电机索引
    uint8_t motor_index = 0;
    uint8_t found = 0;
    for(uint8_t i = 0; i < MAX_GQ_MOTORS; i++)
    {
        if(gq_motor_id_map[i] == Rx_data->id)
        {
            motor_index = i;
            found = 1;
            break;
        }
    }
    // 如果找到了对应的电机ID，解析数据
    if(found)
    {
        // 安全解析（避免对齐问题）
        int16_t pos_raw = (int16_t)((Rx_data->data[3] << 8) | Rx_data->data[2]);
        int16_t vel_raw = (int16_t)((Rx_data->data[5] << 8) | Rx_data->data[4]);
        int16_t tor_raw = (int16_t)((Rx_data->data[7] << 8) | Rx_data->data[6]);
        GQ_Motor_RX[can_port][motor_index].pos = pos_raw * 0.0001f;
        GQ_Motor_RX[can_port][motor_index].vel = vel_raw * 0.00025f;
        GQ_Motor_RX[can_port][motor_index].current = 0.0;
        GQ_Motor_RX[can_port][motor_index].temp = 0.0;
        GQ_Motor_RX[can_port][motor_index].p_int = 0.0;
        GQ_Motor_RX[can_port][motor_index].v_int = 0.0;
        GQ_Motor_RX[can_port][motor_index].t_int = 0.0;
        GQ_Motor_RX[can_port][motor_index].tor = tor_raw * 0.004563f;
    }
}
