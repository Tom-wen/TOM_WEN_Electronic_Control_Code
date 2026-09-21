#include "lerobot.h"

#ifdef COMPILE_LEROBOT
uint8_t ft_motor_count = 0;
// 定义参数结构体
typedef struct {
    uint8_t ids[6];
    float pos[6];
    uint16_t speed[6];
    uint8_t acc[6];
} MyServoCmd;

MyServoCmd servo_cmd;
// 控制函数
void my_control(void *arg)
{
    MyServoCmd *cmd = (MyServoCmd *)arg;
    int16_t pos_int[6];
    for (int i = 0; i < 6; i++)
    {
        pos_int[i] = (int16_t)cmd->pos[i];
    }
    FT_SyncWritePos(cmd->ids, ft_motor_count, pos_int, cmd->speed, cmd->acc);
}

void lerobot_task(void *argument)
{
    Lerobot_Init();
    FT_Motor_Init(&huart8);
    for(;;)
    {
        RemoteControlLerobot();
        FT_Process(my_control, &servo_cmd);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

//机械臂初始化
void Lerobot_Init(void)
{
    servo_cmd.ids[0] = 1;
    servo_cmd.pos[0] = joint1_start;
    servo_cmd.speed[0] = 100;
    servo_cmd.acc[0] = 0;

    servo_cmd.ids[1] = 2;
    servo_cmd.pos[1] = joint2_start;
    servo_cmd.speed[1] = 100;
    servo_cmd.acc[1] = 0;

    servo_cmd.ids[2] = 3;
    servo_cmd.pos[2] = joint3_start;
    servo_cmd.speed[2] = 100;
    servo_cmd.acc[2] = 0;

    servo_cmd.ids[3] = 4;
    servo_cmd.pos[3] = joint4_start;
    servo_cmd.speed[3] = 100;
    servo_cmd.acc[3] = 0;

    servo_cmd.ids[4] = 5;
    servo_cmd.pos[4] = joint5_start;
    servo_cmd.speed[4] = 100;
    servo_cmd.acc[4] = 0;

    servo_cmd.ids[5] = 6;
    servo_cmd.pos[5] = joint6_start;
    servo_cmd.speed[5] = 100;
    servo_cmd.acc[5] = 0;
}

void RemoteControlLerobot(void)
{
    if(sbus_online){
        ft_motor_count = 6;
    }
    else{
        ft_motor_count = 0;
    }
    const int16_t jmin[6] = {joint1_min, joint2_min, joint3_min, joint4_min, joint5_min, joint6_min};
    const int16_t jmax[6] = {joint1_max, joint2_max, joint3_max, joint4_max, joint5_max, joint6_max};
    servo_cmd.pos[0] += rc_ctrl.rc.ch[2] * LEROBOT_STEP / 660.0f;
    servo_cmd.pos[1] += rc_ctrl.rc.ch[3] * LEROBOT_STEP / 660.0f;
    servo_cmd.pos[2] += (-rc_ctrl.rc.ch[1]) * LEROBOT_STEP / 660.0f;
    servo_cmd.pos[3] += rc_ctrl.rc.ch[0] * LEROBOT_STEP / 660.0f;
    servo_cmd.pos[4] += left_knob * LEROBOT_STEP / 660.0f;
    servo_cmd.pos[5] += right_knob * LEROBOT_STEP / 660.0f;
    for (int i = 0; i < 6; i++) {
        if (servo_cmd.pos[i] < jmin[i]) servo_cmd.pos[i] = jmin[i];
        if (servo_cmd.pos[i] > jmax[i]) servo_cmd.pos[i] = jmax[i];
    }
}

#endif
