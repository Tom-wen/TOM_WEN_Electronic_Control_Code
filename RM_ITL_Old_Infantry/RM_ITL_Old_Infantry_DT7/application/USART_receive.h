#ifndef USART_RECEIVE_H
#define USART_RECEIVE_H

#include "struct_typedef.h"




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

extern auto_shoot_t auto_shoot;

/**
 * @brief 用户数据解包
 *
 * @param buf 串口接收数据指针
 * @param auto_shoot 自瞄数据结构指针
 */
void user_data_solve(volatile const uint8_t *buf, auto_shoot_t *auto_shoot);
void send_vision_data(void);

#endif

