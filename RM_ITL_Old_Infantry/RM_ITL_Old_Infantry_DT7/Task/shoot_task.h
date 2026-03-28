/**
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __shoot_TASK_H__
#define __shoot_TASK_H__

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"


//任务初始化 空闲一段时间
#define SHOOT_TASK_INIT_TIME        300
//射击任务控制任务时间函 5ms
#define SHOOT_CONTROL_TIME          5

//射击发射开关通道数据
#ifdef DT7_rc_ctrl 
    #define SHOOT_RC_MODE_CHANNEL       RIGHT_SWITCH
#endif

#ifdef i6x_rc_ctrl
    #define SHOOT_RC_MODE_CHANNEL       1
    #define FRICTION_RC_MODE_CHANNEL    0
#endif


//发射机构pid参数
//摩擦轮
#define FRICTION_KP 30.0f
#define FRICTION_KD 0.0f
#define FRICTION_KI 0.0f
#define FRICTION_MAX_OUT 7000.0f
#define FRICTION_MAX_IOUT 6000.0f
//拨盘
//2006电机拨弹盘的pid
#define TRIGGER_KP     16.0f
#define TRIGGER_KI     0.0f
#define TRIGGER_KD     0.4f
#define TRIGGER_MAX_OUT  9500.0f
#define TRIGGER_MAX_IOUT 6000.0f


//拨弹速度
#define CONTINUE_TRIGGER_SPEED      2500.0f

/*****************************摩擦轮速度*****************************/

#define FRICTION_SPEED_SET    -5500.0f

/*****************************摩擦轮速度*****************************/

#define FRICTION_ACCEL_MAX_OUT      1000.0f //摩擦轮启动加速电流限幅

#define SPEED_WAVE_THRESHOLD        300.0f //计算发射子弹数量的速度波动阈值

//热量预留值，防止超热量
#define SHOOT_HEAT_REMAIN_VALUE 50


typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
} shoot_mode_e;

typedef struct
{
    const motor_measure_t *motor_measure;
    pid_type_def motor_speed_pid;
    float speed;
    float speed_set;
    float angle;
    float set_angle;
    int16_t given_current;
    int8_t ecd_count;
} trigger_motor_t;

typedef struct
{
    const motor_measure_t *motor_measure;
    pid_type_def motor_speed_pid;
    float accel;          //加速度
    float last_speed;    //上次速度
    float speed;          //当前速度
    float speed_set;      //设定速度
    int16_t given_current; //发送电流
} friction_motor_t;

typedef struct
{
    const motor_measure_t *motor_measure;
    pid_type_def motor_speed_pid;
    float accel;          //加速度
    float speed;          //当前速度
    float speed_set;      //设定速度
    int16_t given_current; //发送电流
} avoid_shoot_motor_t;


typedef struct
{
    shoot_mode_e shoot_mode;

    trigger_motor_t trigger_motor;
    friction_motor_t right_fricition_motor;
    friction_motor_t left_fricition_motor;

    uint16_t shoot_count;//发射计数

} shoot_control_t;

void shoot_task(void const *pvParameters);

extern shoot_control_t shoot_control;

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/