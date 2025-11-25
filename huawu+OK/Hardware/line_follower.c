#include "stm32f10x.h"
#include "headfile.h"
#include "gray_line.h"
#include "pid.h"
#include "Motor.h"

// 控制参数
#define BASE_SPEED 400     // 基础速度
#define MAX_TURN_OFFSET 300 // 最大转向偏移量

// 全局变量
float current_speed = 0.0f;
float target_speed = BASE_SPEED;

// 初始化函数
void line_follower_init(void)
{
    // 初始化硬件
    gray_init();
    Motor_Init();
    
    // 初始化PID控制器
    pid_init(&line_pid, POSITION_PID, line_kp, line_ki, line_kd);
    pid_init(&speed_pid, POSITION_PID, speed_kp, speed_ki, speed_kd);
}

// 主巡线控制函数
void line_following_control(void)
{
    float line_position, turn_adjust, left_speed, right_speed;
    
    // 1. 获取线路位置
    line_position = get_line_position();
    
    // 2. 巡线PID计算转向调整量
    turn_adjust = line_pid_control(line_position);
    
    // 3. 限幅转向调整量
    if(turn_adjust > MAX_TURN_OFFSET) turn_adjust = MAX_TURN_OFFSET;
    if(turn_adjust < -MAX_TURN_OFFSET) turn_adjust = -MAX_TURN_OFFSET;
    
    // 4. 计算左右轮速度（差速控制）
    left_speed = target_speed + turn_adjust;
    right_speed = target_speed - turn_adjust;
    
    // 5. 速度限幅
    if(left_speed > 800) left_speed = 800;
    if(left_speed < 0) left_speed = 0;
    if(right_speed > 800) right_speed = 800;
    if(right_speed < 0) right_speed = 0;
    
    // 6. 设置电机速度
    // 注意：需要根据你的电机驱动函数调整
    // 假设有设置左右轮速度的函数
    set_motor_speeds(left_speed, right_speed);
}

// 设置目标速度
void set_target_speed(float speed)
{
    target_speed = speed;
    if(target_speed > 800) target_speed = 800;
    if(target_speed < 0) target_speed = 0;
}

// 停止小车
void stop_car(void)
{
    set_target_speed(0);
    // 停止电机
    set_motor_speeds(0, 0);
}

// 检测是否丢失线路
uint8_t is_line_lost(void)
{
    // 如果所有传感器都检测不到黑线，认为丢失线路
    if(D1==0 && D2==0 && D3==0 && D4==0 && D5==0 && D6==0 && D7==0 && D8==0) {
        return 1;
    }
    return 0;
}

// 丢失线路处理
void handle_line_lost(void)
{
    // 停止或减速
    set_target_speed(BASE_SPEED / 2);
    // 可以添加寻线算法，比如原地旋转寻找线路
}