#ifndef __TEST_H__
#define __TEST_H__

#include "Init.h"
#include "bsp_spi.h"
#include "Signal.h"

#ifdef COMPILE_TEST
typedef enum
{
    test_ZERO_FORCE = 0,    // 电流零输入
    test_ROTATE,            // 小陀螺模式
    test_NO_FOLLOW,         // 不跟随，允许全向平移
    test_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} test_mode_e;

//底盘控制数据
typedef struct
{
    // 控制部分
    float vx;                          // 前进方向速度
    float vy;                          // 横移方向速度
    float w;                           // 旋转角速度
    float wheel3508_rpm[4];            // 3508速度
    PID cascade_pid[3];                   // 底盘跟随云台旋转PID
    test_mode_e test_mode;
} test_Ctrl_Cmd_s;

void test_task(void *argument);
void test_Init(MotorInstance *motors, test_Ctrl_Cmd_s *test_Cmd);
void test_Motor_Status(Motor_status status, MotorInstance *motors, uint8_t motor_count);
void test_mode_update(test_Ctrl_Cmd_s *test_Cmd);
void test_motor_updata(MotorInstance *motors, MotorInstance *motors2, MotorInstance *motors3);
void RemoteControltest(test_Ctrl_Cmd_s *test_Cmd);
void test_PIDClear(MotorInstance  *motors);
#endif

#endif
