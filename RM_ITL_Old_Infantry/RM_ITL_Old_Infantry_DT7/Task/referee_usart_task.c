#include "referee_usart_task.h"

extern osMessageQId uart_queueHandle;
//裁判系统数据全局实例
referee_receive referee_data;
uint16_t referee_cmd;

void referee_usart_task(void *argument)
{
    REFEREE_RX_TypeDef *pRecvUartData = NULL; /* 定义指向串口数据的指针 */
    REFEREE_RX_TypeDef *pRecvUartData1 = NULL; /* 定义指向串口数据的指针 */
    for(;;)
    {
        referee_usart_analyse(pRecvUartData, &referee_data);
        vTaskDelay(pdMS_TO_TICKS(2));
    }

}

/**
 * @brief 裁判系统数据解析
 * @param pRecvUartData 接收到的串口数据指针
 */
void referee_usart_analyse(REFEREE_RX_TypeDef *pRecvUartData, referee_receive *referee_data)
{
    // 从队列获取数据
        referee_routine_analyse(referee_data);

}

/**
 * @brief 常规链路数据解析（从环形缓冲区读取）
 * @param referee_data 裁判系统数据结构体指针
 */
void referee_routine_analyse(referee_receive *referee_data)
{
    static uint8_t frame_buffer[256];  // 静态缓冲区，用于存储从环形缓冲区读取的数据帧
    uint16_t frame_length;
    uint16_t cmd_id;
    uint16_t data_length;
    
    // 从环形缓冲区获取完整帧
    frame_length = Referee_GetFrame(frame_buffer);
    
    // 如果没有获取到完整帧，直接返回
    if (frame_length == 0) {
        return;
    }
    
    // 提取cmd_id（小端字节序，从buffer[5]开始）
    cmd_id = frame_buffer[5] | (frame_buffer[6] << 8);
    
    // 提取data_length（小端字节序，从buffer[1]开始）
    data_length = frame_buffer[1] | (frame_buffer[2] << 8);
    
    // 根据cmd_id解析不同数据
    switch (cmd_id)
    {
        case CMD_ID_ROBOT_STATUS:  // 0x0201: 机器人性能体系数据
        {
            // 检查数据长度是否正确（robot_status_t为13字节）
            if (data_length == 13)
            {
                // 解析robot_status_t数据（从buffer[7]开始）
                referee_data->robot.robot_id = frame_buffer[7];
                referee_data->robot.robot_level = frame_buffer[8];
                referee_data->robot.current_HP = frame_buffer[9] | (frame_buffer[10] << 8);
                referee_data->robot.maximum_HP = frame_buffer[11] | (frame_buffer[12] << 8);
                referee_data->robot.shooter_barrel_cooling_value = frame_buffer[13] | (frame_buffer[14] << 8);
                referee_data->robot.shooter_barrel_heat_limit = frame_buffer[15] | (frame_buffer[16] << 8);
                referee_data->robot.chassis_power_limit = frame_buffer[17] | (frame_buffer[18] << 8);
                referee_data->robot.power_management_gimbal_output = (frame_buffer[19] >> 0) & 0x01;
                referee_data->robot.power_management_chassis_output = (frame_buffer[19] >> 1) & 0x01;
                referee_data->robot.power_management_shooter_output = (frame_buffer[19] >> 2) & 0x01;
            }
            break;
        }
        case CMD_ID_POWER_HEAT:  // 0x0202: 实时底盘缓冲能量和射击热量数据
        {
            // 检查数据长度是否正确（14字节）
            if (data_length == 14)
            {
                // 解析power_heat_data_t数据
                uint16_t *data_ptr = (uint16_t *)&frame_buffer[7];
                
                referee_data->power_heat.reserved_1 = data_ptr[0];              // buffer[7-8]
                referee_data->power_heat.reserved_2 = data_ptr[1];              // buffer[9-10]
                memcpy(&referee_data->power_heat.reserved_3, &frame_buffer[11], 4);  // buffer[11-14] float
                referee_data->power_heat.buffer_energy = frame_buffer[15] | (frame_buffer[16] << 8);  // buffer[15-16]
                referee_data->power_heat.shooter_17mm_1_barrel_heat = frame_buffer[17] | (frame_buffer[18] << 8);  // buffer[17-18]
                referee_data->power_heat.shooter_42mm_barrel_heat = frame_buffer[19] | (frame_buffer[20] << 8);  // buffer[19-20]
            }
            break;
        }
        case CMD_ID_ROBOT_POS:  // 0x0203: 机器人位置数据
        {
            // 检查数据长度是否正确（12字节）
            if (data_length == 12)
            {
                // 解析robot_pos_t数据（3个float，小端字节序）
                memcpy(&referee_data->robot_pos.x, &frame_buffer[7], 4);      // buffer[7-10]
                memcpy(&referee_data->robot_pos.y, &frame_buffer[11], 4);     // buffer[11-14]
                memcpy(&referee_data->robot_pos.angle, &frame_buffer[15], 4);   // buffer[15-18]
            }
            break;
        }
        case CMD_ID_BUFF:  // 0x0204: 机器人增益和底盘能量数据
        {
            // 检查数据长度是否正确（8字节）
            if (data_length == 8)
            {
                // 解析buff_t数据
                referee_data->buff.recovery_buff = frame_buffer[7];  // buffer[7]
                referee_data->buff.cooling_buff = frame_buffer[8] | (frame_buffer[9] << 8);  // buffer[8-9]
                referee_data->buff.defence_buff = frame_buffer[10];  // buffer[10]
                referee_data->buff.vulnerability_buff = frame_buffer[11];  // buffer[11]
                referee_data->buff.attack_buff = frame_buffer[12] | (frame_buffer[13] << 8);  // buffer[12-13]
                referee_data->buff.remaining_energy = frame_buffer[14];  // buffer[14]
            }
            break;
        }
        case CMD_ID_HURT:  // 0x0206: 伤害状态数据
        {
            // 检查数据长度是否正确（1字节）
            if (data_length == 1)
            {
                // 解析hurt_data_t数据（bit位操作）
                referee_data->hurt.armor_id = frame_buffer[7] & 0x0F;                    // 低4位
                referee_data->hurt.HP_deduction_reason = (frame_buffer[7] >> 4) & 0x0F;  // 高4位
            }
            break;
        }
        case CMD_ID_SHOOT:  // 0x0207: 实时射击数据
        {
            // 检查数据长度是否正确（shoot_data_t为7字节）
            if (data_length == 7)
            {
                // 解析shoot_data_t数据（从frame_buffer[7]开始）
                referee_data->shoot_data.bullet_type = frame_buffer[7];
                referee_data->shoot_data.shooter_number = frame_buffer[8];
                referee_data->shoot_data.launching_frequency = frame_buffer[9];

                // 解析float类型的初速度（小端字节序）
                memcpy(&referee_data->shoot_data.initial_speed, &frame_buffer[10], 4);
            }
            break;
        }        
        case CMD_ID_PROJECTILE_ALLOWANCE:  // 0x0208: 允许发弹量数据
        {
            // 检查数据长度是否正确（8字节）
            if (data_length == 8)
            {
                // 解析projectile_allowance_t数据（4个uint16_t）
                referee_data->projectile_allowance.projectile_allowance_17mm = frame_buffer[7] | (frame_buffer[8] << 8);  // buffer[7-8]
                referee_data->projectile_allowance.projectile_allowance_42mm = frame_buffer[9] | (frame_buffer[10] << 8);  // buffer[9-10]
                referee_data->projectile_allowance.remaining_gold_coin = frame_buffer[11] | (frame_buffer[12] << 8);  // buffer[11-12]
                referee_data->projectile_allowance.projectile_allowance_fortress = frame_buffer[13] | (frame_buffer[14] << 8);  // buffer[13-14]
            }
            break;
        }
        case CMD_ID_RFID:  // 0x0209: RFID模块状态数据
        {
            // 检查数据长度是否正确（5字节）
            if (data_length == 5)
            {
                // 解析rfid_status_t数据
                referee_data->rfid_status.rfid_status = frame_buffer[7] | (frame_buffer[8] << 8) | 
                                                         (frame_buffer[9] << 16) | 
                                                         (frame_buffer[10] << 24);  // buffer[7-10]
                referee_data->rfid_status.rfid_status_2 = frame_buffer[11];  // buffer[11]
            }
            break;
        }
        case CMD_ID_GROUND_ROBOT_POS:  // 0x020B: 地面机器人位置数据
        {
            // 检查数据长度是否正确（40字节）
            if (data_length == 40)
            {
                // 解析ground_robot_position_t数据（10个float，小端字节序）
                memcpy(&referee_data->ground_robot_pos.hero_x, &frame_buffer[7], 4);    // buffer[7-10]
                memcpy(&referee_data->ground_robot_pos.hero_y, &frame_buffer[11], 4);    // buffer[11-14]
                memcpy(&referee_data->ground_robot_pos.engineer_x, &frame_buffer[15], 4); // buffer[15-18]
                memcpy(&referee_data->ground_robot_pos.engineer_y, &frame_buffer[19], 4); // buffer[19-22]
                memcpy(&referee_data->ground_robot_pos.standard_3_x, &frame_buffer[23], 4); // buffer[23-26]
                memcpy(&referee_data->ground_robot_pos.standard_3_y, &frame_buffer[27], 4); // buffer[27-30]
                memcpy(&referee_data->ground_robot_pos.standard_4_x, &frame_buffer[31], 4); // buffer[31-34]
                memcpy(&referee_data->ground_robot_pos.standard_4_y, &frame_buffer[35], 4); // buffer[35-38]
                memcpy(&referee_data->ground_robot_pos.reserved_1, &frame_buffer[39], 4);  // buffer[39-42]
                memcpy(&referee_data->ground_robot_pos.reserved_2, &frame_buffer[43], 4);  // buffer[43-46]
            }
            break;
        }
        case CMD_ID_SENTRY_INFO:  // 0x020D: 哨兵自主决策信息同步数据
        {
            // 检查数据长度是否正确（6字节）
            if (data_length == 6)
            {
                // 解析sentry_info_t数据
                referee_data->sentry_info.sentry_info = frame_buffer[7] | (frame_buffer[8] << 8) | 
                                                          (frame_buffer[9] << 16) | 
                                                          (frame_buffer[10] << 24);  // buffer[7-10]
                referee_data->sentry_info.sentry_info_2 = frame_buffer[11] | (frame_buffer[12] << 8);  // buffer[11-12]
            }
            break;
        }
        default:
            // 未识别的命令码，忽略
            referee_cmd = cmd_id;
            break;
    }
}
