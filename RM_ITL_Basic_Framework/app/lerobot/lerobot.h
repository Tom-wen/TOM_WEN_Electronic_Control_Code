#ifndef __LEROBOT_H__
#define __LEROBOT_H__

#include "Init.h"

#ifdef COMPILE_LEROBOT

//各个关节软件限位
#define joint1_min      1664
#define joint1_max      2300
#define joint1_start    1989
#define joint2_min      829
#define joint2_max      2370
#define joint2_start    829
#define joint3_min      818
#define joint3_max      3089
#define joint3_start    3089
#define joint4_min      788
#define joint4_max      2914
#define joint4_start    788
#define joint5_min      75
#define joint5_max      2069
#define joint5_start    1011
#define joint6_min      914
#define joint6_max      2107
#define joint6_start    1307

//舵机步进系数
#define LEROBOT_STEP  0.9f

void lerobot_task(void *argument);
void Lerobot_Init(void);
void RemoteControlLerobot(void);

#endif

#endif

