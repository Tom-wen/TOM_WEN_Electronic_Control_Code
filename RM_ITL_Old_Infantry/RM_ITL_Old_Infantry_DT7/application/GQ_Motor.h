#ifndef __GQ_MOTOR_H
#define __GQ_MOTOR_H

#include "main.h"

typedef struct
{
  uint16_t id;
  float position;
  float velocity; 
  float torque;
} GQ_Motor_Measure_t;
extern GQ_Motor_Measure_t GQ_Motor_Measure[4];

#define CAN_GQ_M1_ID 0x100
#define CAN_GQ_M2_ID 0x200
#define CAN_GQ_M3_ID 0x300
#define CAN_GQ_M4_ID 0x400

void motor_control_current(uint8_t id, uint16_t cur);
void get_GQ_motor_measure(GQ_Motor_Measure_t *motor, uint8_t *data, uint16_t id);
void timed_return_motor_status(uint8_t id, int16_t t_ms);
void GQ_Motor_send_current(int16_t cur_1, int16_t cur_2, int16_t cur_3, int16_t cur_4);


#endif