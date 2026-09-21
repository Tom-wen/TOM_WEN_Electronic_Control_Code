#include "ui_task.h"

#ifdef COMPILE_UI

void ui_task(void *argument)
{
    ui_init_static_ui_Ungroup();
    ui_init_static_ui_group4();
    ui_init_static_ui_group5();
    ui_init_static_ui_group7();
    ui_init_mode_ui_group1();
    ui_init_mode_ui_group2();
    ui_init_static_ui_group8();
    for(;;)
    {
        ui_self_id = referee_data.robot.robot_id;
        //按键b刷新UI
        if(rc_ctrl.key_rising_edge & (1 << 15)){
            ui_init_static_ui_Ungroup();
            ui_init_static_ui_group4();
            ui_init_static_ui_group5();
            ui_init_static_ui_group7();
            ui_init_mode_ui_group1();
            ui_init_mode_ui_group2(); 
            ui_init_static_ui_group8();          
        }
        dynamic_ui_speed_update();
        dynamic_ui_mode_update();
        dynamic_ui_aim_mode_update();
        dynamic_ui_pitch_update(&INS);
        dynamic_ui_power_update();
        dynamic_ui_gimbal_update(&INS);
        dynamic_ui_fire_update();
        vTaskDelay(pdMS_TO_TICKS(2));
    }

}

void dynamic_ui_speed_update()
{
    switch (chassis_cmd_send.gear_level)
    {
        case 1:
            ui_mode_ui_group2_speed_number->end_y = 660;
            ui_update_mode_ui_group2();
            break;
        case 2:
            ui_mode_ui_group2_speed_number->end_y = 720;
            ui_update_mode_ui_group2();
            break;    
        case 3:
            ui_mode_ui_group2_speed_number->end_y = 780;
            ui_update_mode_ui_group2();
            break;               
        default:
            ui_mode_ui_group2_speed_number->end_y = 660;
            ui_update_mode_ui_group2();
            break;
    }
}

//底盘控制模式更新
void dynamic_ui_mode_update()
{
    switch (chassis_cmd_send.chassis_mode)
    {
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            ui_mode_ui_group1_follow_mode->color = 2;
            ui_mode_ui_group1_rotate_mode->color = 8;
            ui_update_mode_ui_group1();
            break;
        case CHASSIS_ROTATE:
            ui_mode_ui_group1_follow_mode->color = 8;
            ui_mode_ui_group1_rotate_mode->color = 2;
            ui_update_mode_ui_group1();
            break;    
        default:
            break;
    }
}

//发射控制模式更新
void dynamic_ui_aim_mode_update()
{
    if(auto_aim_flag){
        ui_mode_ui_group1_auto_arm->color = 2;
        ui_update_mode_ui_group1();
    }
    else{
        ui_mode_ui_group1_auto_arm->color = 8;
        ui_update_mode_ui_group1();        
    }
}
//pitch角度更新
void dynamic_ui_pitch_update(INS_t *ins)
{
    float pitch = ins->Pitch;
    
    // 限幅 ±30°
    if (pitch > 30.0f) pitch = 30.0f;
    if (pitch < -30.0f) pitch = -30.0f;
    
    // 弧形刻度参数（从UI刻度位置拟合得出）
    // 弧心约 (931, 540)，指示器半径约 364
    // pitch角度到弧角度缩放比约 1.37
    float arc_angle = pitch * 1.37f * 3.14159265f / 180.0f;
    
    // 计算指示器中心在弧上的位置
    float center_x = 931.0f + 364.0f * cosf(arc_angle);
    float center_y = 540.0f + 364.0f * sinf(arc_angle);
    
    // 线段长度保持22像素，水平放置
    ui_static_ui_group5_current_angle->start_x = (int16_t)(center_x - 11.0f);
    ui_static_ui_group5_current_angle->end_x   = (int16_t)(center_x + 11.0f);
    ui_static_ui_group5_current_angle->start_y  = (int16_t)center_y;
    ui_static_ui_group5_current_angle->end_y    = (int16_t)center_y;
    ui_update_static_ui_group5();
}

// 功率UI显示
void dynamic_ui_power_update(void)
{
    float level = (float)super_cap.capacitor_level;
    
    // 限幅 0~100
    if (level < 0.0f) level = 0.0f;
    if (level > 100.0f) level = 100.0f;
    
    // 线性映射: 0% → angle 232, 100% → angle 270
    ui_static_ui_group4_power->start_angle = 232;
    ui_static_ui_group4_power->end_angle = (int16_t)(233.0f + (level / 100.0f) * 38.0f);
    ui_update_static_ui_group4();
}


// 云台姿态UI显示
void dynamic_ui_gimbal_update(INS_t *ins)
{
    float yaw = ins->Yaw; // -180° ~ +180°
    
    // 圆心和线长（0°时线条向下，长度69像素）
    float cx = 245.0f;
    float cy = 620.0f;
    float length = 69.0f;
    
    // 角度转弧度
    float yaw_rad = yaw * 3.14159265f / 180.0f;
    
    // 计算旋转后的终点坐标
    // 0°向下，顺时针为正角度
    ui_static_ui_group7_gimbal->end_x = (int16_t)(cx + length * sinf(yaw_rad));
    ui_static_ui_group7_gimbal->end_y = (int16_t)(cy + length * cosf(yaw_rad));
    
    ui_update_static_ui_group7();
}
//开火UI
void dynamic_ui_fire_update()
{
    if(fire_flag == 1){
        ui_static_ui_group8_FIRE->color = 2;
    }
    else{
        ui_static_ui_group8_FIRE->color = 8;
    }
    ui_update_static_ui_group8();
}


#endif
