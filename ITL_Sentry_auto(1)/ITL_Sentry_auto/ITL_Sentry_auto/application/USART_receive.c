/**
 * @file USART_receive.c
 * @author 陈冠华
 * @brief 通过串口中断接收数据，实现对主控板与上位机或其他设备的通讯
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "USART_receive.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"
#include "referee.h"
#include "usart.h"
#include "chassis_task.h"
#include "chassis_power_control.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "usbd_cdc_if.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "chassis_behaviour.h"

extern chassis_move_t chassis_move;
extern UART_HandleTypeDef huart1;
extern float current;
extern gimbal_control_t gimbal_control;
auto_shoot_t auto_shoot = {{0xFF, 0xFE}, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0x00}; // 自动射击数据
auto_move_t auto_move = {0.0f, 0.0f, 0};//导航数据
user_send_data_t user_send_data;
int16_t pitch_add, yaw_add;
extern quaternion_t quaternion;
static float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// 串口接收原始缓冲区，为10个字节，总共2个18字节缓冲区，防止DMA接收越界
uint8_t usart1_rx_buf[2][USART1_RX_BUF_NUM];

void user_usart_init(void)
{
    usart1_init(usart1_rx_buf[0], usart1_rx_buf[1], USART1_RX_BUF_NUM);
}

// void USART1_IRQHandler(void)
// {
//     if (huart1.Instance->SR & UART_FLAG_RXNE) // 接收到数据
//     {
//         __HAL_UART_CLEAR_PEFLAG(&huart1);
//     }
//     else if (USART1->SR & UART_FLAG_IDLE)
//     {
//         static uint16_t this_time_rx_len = 0;

//         __HAL_UART_CLEAR_PEFLAG(&huart1);

//         if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
//         {
//             /* Current memory buffer used is Memory 0 */
//             // 失效DMA
//             __HAL_DMA_DISABLE(huart1.hdmarx);
//             // 获取此次接收到的数据长度, 长度 = 设定长度 - 剩余长度
//             this_time_rx_len = USART1_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;
//             // 重置设定缓冲区长度
//             huart1.hdmarx->Instance->NDTR = USART1_RX_BUF_NUM;
//             // 设定缓冲区为1
//             huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
//             // 使能DMA
//             __HAL_DMA_ENABLE(huart1.hdmarx);

//             if (this_time_rx_len == USER_FRAME_LENGTH)
//             {
//                 // 解析数据
//                 user_data_solve(usart1_rx_buf[0], &auto_shoot);
//             }
//         }
//         else
//         {
//             /* Current memory buffer used is Memory 1 */
//             // 失效DMA
//             __HAL_DMA_DISABLE(huart1.hdmarx);
//             // 获取此次接收到的数据长度, 长度 = 设定长度 - 剩余长度
//             this_time_rx_len = USART1_RX_BUF_NUM - huart1.hdmarx->Instance->NDTR;
//             // 重置设定缓冲区长度
//             huart1.hdmarx->Instance->NDTR = USART1_RX_BUF_NUM;
//             // 设定缓冲区为0
//             huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
//             // 使能DMA
//             __HAL_DMA_ENABLE(huart1.hdmarx);
//             if (this_time_rx_len == USER_FRAME_LENGTH)
//             {
//                 // 解析数据
//                 user_data_solve(usart1_rx_buf[1], &auto_shoot); // 1
//             }
//         }
//     }
// }

/**
 * @brief 用户数据解析函数
 *
 * @param buf 待解析数据的缓冲区指针
 * @param auto_shoot 自动射击数据结构指针
 */
void user_auto_shoot_data_solve(const uint8_t *buf, auto_shoot_t *auto_shoot)
{
    // 检查帧头和帧尾
    if (buf[0] == 'V' && buf[1] == 'G' && buf[27] == 'V')
    {
        // 射击模式
        auto_shoot->mode = buf[2];

        // 解析float数据 (协议为小端序)
        // yaw角度
        memcpy(&auto_shoot->yaw_add, &buf[3], sizeof(float));
        // yaw角速度
        memcpy(&auto_shoot->yaw_vel, &buf[7], sizeof(float));
        // yaw角加速度
        memcpy(&auto_shoot->yaw_acc, &buf[11], sizeof(float));
        // pitch角度
        memcpy(&auto_shoot->pitch_add, &buf[15], sizeof(float));
        // pitch角速度
        memcpy(&auto_shoot->pitch_vel, &buf[19], sizeof(float));
        // pitch角加速度
        memcpy(&auto_shoot->pitch_acc, &buf[23], sizeof(float));

        detect_hook(USER_USART_DATA_TOE);
    }
    else
    {
        // auto_shoot->pitch_add = 0.1f; // 测试
        // 数据错误
    }
}

void send_vision_data(void)
{
    uint8_t sendBuffer[256]; // 确保足够大
    uint16_t offset = 0;

    INS_quat[0] = quaternion.q0;
    INS_quat[1] = quaternion.q1;
    INS_quat[2] = quaternion.q2;
    INS_quat[3] = quaternion.q3;
    // 帧头
    sendBuffer[offset++] = 'G';
    sendBuffer[offset++] = 'V';

    // mode
    sendBuffer[offset++] = 1; // 自动模式

    // 打包INS_quat四元数数据 (16字节)
    memcpy(&sendBuffer[offset], &INS_quat[0], sizeof(float));
    offset += 4;
    memcpy(&sendBuffer[offset], &INS_quat[1], sizeof(float));
    offset += 4;
    memcpy(&sendBuffer[offset], &INS_quat[2], sizeof(float));
    offset += 4;
    memcpy(&sendBuffer[offset], &INS_quat[3], sizeof(float));
    offset += 4;

    // yaw (4字节)
    memcpy(&sendBuffer[offset], &gimbal_control.gimbal_yaw_motor.absolute_angle, 4);
    offset += 4;

    // yaw角速度 (4字节)
    memcpy(&sendBuffer[offset], 0, 4);
    offset += 4;

    // pitch (4字节)
    float pitch_negative = gimbal_control.gimbal_pitch_motor.absolute_angle;
    memcpy(&sendBuffer[offset], &pitch_negative, 4);
    offset += 4;

    // pitch角速度 (4字节)
    memcpy(&sendBuffer[offset], 0, 4);
    offset += 4;

    // 子弹速度 (4字节)
    memcpy(&sendBuffer[offset], 0, 4);
    offset += 4;

    // 子弹计数 (2字节)
    memcpy(&sendBuffer[offset], 0, 2);
    offset += 2;

    // 帧尾
    sendBuffer[offset++] = 'G';

    // 发送
    CDC_Transmit_FS(sendBuffer, offset);
}

/**
 * @brief 用户数据解包
 *
 * @param buf 串口接收数据指针
 * @param auto_shoot 自瞄数据结构指针
 */
extern uint8_t target_distance;


void user_data_solve(volatile const uint8_t *buf, auto_shoot_t *auto_shoot,auto_move_t *auto_move)
{
    //校验
    if (buf[0] == 0xFF && buf[11]== 0xFE)
    {
				aRGB_led_show(0xFF00FF00);
        auto_shoot->pitch_add =0.001f *((float)(buf[2] << 8 | buf[1]));//(0.0001f * ((float)(buf[2] << 8 | buf[1]))) - USART_PI;
        auto_shoot->yaw_add = 0.001f *((float)(buf[4] << 8 | buf[3]));//(0.0001f * ((float)(buf[4] << 8 | buf[3]))) - USART_PI;
			if((auto_shoot->pitch_add||auto_shoot->yaw_add)!=0)
				{
					 find_target=1;
				}
				else
				{
					find_target=0;
				}
			switch(buf[5])
        {
            case 0:auto_shoot->pitch_add=auto_shoot->pitch_add*-1;auto_shoot->yaw_add=auto_shoot->yaw_add*-1;break;
            case 1:auto_shoot->pitch_add=auto_shoot->pitch_add*-1;break;
            case 2:auto_shoot->yaw_add=auto_shoot->yaw_add*-1;break;
            default:break;
        }
        target_distance=buf[6];
				auto_move->auto_vx=buf[7];//((float)(buf[8] << 8 | buf[7]));
				auto_move->auto_vy=buf[8];//((float)(buf[10] << 8 | buf[9]));
				auto_move->auto_wz=buf[9];//((float)(buf[12] << 8 | buf[11]));
				switch(buf[10])
        {
            case 0:auto_move->auto_vx=auto_move->auto_vx*-1;auto_move->auto_vy=auto_move->auto_vy*-1;auto_move->auto_wz=auto_move->auto_wz*-1;break;//xyz为负
            case 1:auto_move->auto_vx=auto_move->auto_vx*-1;auto_move->auto_vy=auto_move->auto_vy*-1;break;//xy为负
            case 2:auto_move->auto_vx=auto_move->auto_vx*-1;auto_move->auto_wz=auto_move->auto_wz*-1;break;//xz
						case 3:auto_move->auto_vy=auto_move->auto_vy*-1;auto_move->auto_wz=auto_move->auto_wz*-1;break;//yz
						case 4:auto_move->auto_vx=auto_move->auto_vx*-1;break;//x
						case 5:auto_move->auto_vy=auto_move->auto_vy*-1;break;//y
						case 6:auto_move->auto_wz=auto_move->auto_wz*-1;break;//z
//						case 7:
//						case 8:auto_move_flag=AUTO_CHASSIS_TOP;
            default:break;
        }
				if((auto_move->auto_vx||auto_move->auto_vy||auto_move->auto_wz)!=0)//有移动数据
				{
					auto_move_flag=AUTO_MOVE;//导航
					//find_target=0;//走路不自瞄,头不动
				}
//				else if((auto_move->auto_vx&&auto_move->auto_vy&&auto_move->auto_wz&&auto_shoot->pitch_add&&auto_shoot->yaw_add==0)&&buf[10]==8)//&&buf[10]==8
//				{
//					auto_move_flag=AUTO_CHASSIS_TOP;
//				}
				else
				{
					auto_move_flag=AUTO_CHASSIS_TOP;
					//auto_move_flag=AUTO_IN_HOME;
				}
        detect_hook(USER_USART_DATA_TOE);
    }
}



int m = 1;//模式切换标志位，为0表示传输电机参数，为1表示传输云台数据,2输出功率
//bool_t reset_tracker = 0;//重置检测标志位，默认0
float chassis_power1 = 0.0f;        //底盘功率
float chassis_power_buffer1 = 0.0f;   //底盘缓冲能量
uint8_t back_home=0;
extern ext_power_heat_data_t power_heat_data_t;
extern void get_chassis_power_and_buffer(float *current,float *volt,float *power, float *buffer);
extern INS_data_t INS_data;
extern ext_game_robot_state_t robot_state;


void float_to_be_bytes(float value, uint8_t* bytes) {
    uint32_t int_val;
    memcpy(&int_val, &value, sizeof(float));
    
    bytes[0] = (int_val >> 24) & 0xFF;
    bytes[1] = (int_val >> 16) & 0xFF;
    bytes[2] = (int_val >> 8) & 0xFF;
    bytes[3] = int_val & 0xFF;
}

void user_data_pack_handle()
{
    if(m==0)
    {
    //发送当前底盘速度 单位m/s
        ANO_DT_send_int16((int16_t)(chassis_move.motor_chassis[0].speed*100), (int16_t)(chassis_move.motor_chassis[1].speed*100),
        (int16_t)(chassis_move.motor_chassis[2].speed*100), (int16_t)(chassis_move.motor_chassis[3].speed*100), 
        power_heat_data_t.chassis_current, power_heat_data_t.chassis_power,power_heat_data_t.chassis_power_buffer, 0 );
//      get_chassis_power_and_buffer(&chassis_power1, &chassis_power_buffer1);
//      ANO_DT_send_int16((int16_t)chassis_power1, (int16_t)chassis_power_buffer1	,0, 0, 0, 0, 0, 0 );
    }
    else
    {
				static uint8_t tx_buf[4];
        if (get_robot_id() <= 7)
        {
            user_send_data.enemy_color = 0;//红色
        }
        else
        {
            user_send_data.enemy_color = 1;//蓝色
        }
				if(robot_state.remain_HP>=robot_state.max_HP*50/100)
				{
					back_home=0;
				}
				else
				{
					back_home=1;
				}
        tx_buf[0] = 0xFF;
				tx_buf[1] = user_send_data.enemy_color;
				tx_buf[2] = back_home;
				tx_buf[3] = 0xFE;
       usart1_tx_dma_enable(tx_buf, 4);
//  if(a==2)
//  {
//      //get_chassis_power_and_buffer(&chassis_power1, &chassis_power_buffer1);
//      ANO_DT_send_int16((int16_t)chassis_power1, (int16_t)chassis_power_buffer1,0, 0, 0, 0, 0, 0 );
//      }
    }
}