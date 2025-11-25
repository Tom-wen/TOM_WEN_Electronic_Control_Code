/**
 * @file USART_receive.h
 * @author 何清华
 * @brief 串口中断接收函数，用来处理单片机与其他设备的串口通信数据
 * @version 0.1
 * @date 2022-03-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef USART_RECEIVE_H
#define USART_RECEIVE_H

#include "struct_typedef.h"

#define USART1_RX_BUF_NUM 18u
#define USER_FRAME_LENGTH 6u

#define USART_PI 3.1416f

typedef struct
{
  uint8_t head[2];
  uint8_t mode ;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw_add;
  float yaw_vel;
  float yaw_acc;
  float pitch_add;
  float pitch_vel;
  float pitch_acc;
  uint8_t tail;
} auto_shoot_t;

typedef struct
{
    float *gimbal_pitch_angle;
    float *gimbal_yaw_gyro;
    uint8_t enemy_color;
} user_send_data_t;

extern auto_shoot_t auto_shoot;
extern user_send_data_t user_send_data;
void user_usart_init(void);
extern void user_data_pack_handle(void);

#endif
