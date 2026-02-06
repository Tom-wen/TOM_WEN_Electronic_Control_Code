#include "hoisting_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "GQ_Motor.h"
#include "pid.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "bsp_led.h"
#include "bsp_buzzer.h"


hoisting_t hoisting_control;

static void hoisting_init(void);
static void hoisting_feedback_update(void);
static void hoisting_control_loop(void);
static void hoisting_behavour_set(void);
static void hoisting_set_control(void);
static void hoisting_rc_to_speed(float *forward, float *backward);
static void hoisting_update_offline_state(void);
static void hoisting_stall_protection(void); 
static uint8_t hoisting_check_stall_state(void);  

void hoisting_task(void const *pvParameters)
{
  //空闲一段时间
  vTaskDelay(HOISTING_TASK_INIT_TIME);

  hoisting_init();
  while(1)
  {
    hoisting_behavour_set();

    hoisting_feedback_update();
    hoisting_set_control();

    hoisting_control_loop();

    if (!(toe_is_error(HOISTING_MOTOR1_TOE) && toe_is_error(HOISTING_MOTOR2_TOE) && 
          toe_is_error(HOISTING_MOTOR3_TOE) && toe_is_error(HOISTING_MOTOR4_TOE)))
    {
          if (toe_is_error(DBUS_TOE))  // 遥控器离线
          {
            hoisting_update_offline_state();
            GQ_Motor_send_current(0,0,0,0);
          }
          else
          {
                hoisting_stall_protection();//堵转保护	
                if (fabsf(GQ_Motor_Measure[0].position) < 3 && fabsf(GQ_Motor_Measure[1].position) < 3 
                    && fabsf(GQ_Motor_Measure[2].position) < 3 && fabsf(GQ_Motor_Measure[3].position) < 3)
                {
                      if (hoisting_check_stall_state())  // 检查堵转状态
                      {
                          buzzer_warning();  // 堵转保护模式
                          LED_Warning();//堵转保护模式
                          GQ_Motor_send_current(0,0,0,0);
                      }
                      else
                      {
                          GQ_Motor_send_current(
                            hoisting_control.hoisting_motor[0].given_current,hoisting_control.hoisting_motor[1].given_current,
                            hoisting_control.hoisting_motor[2].given_current,hoisting_control.hoisting_motor[3].given_current);
                      }
                } 
                else
                {
                    LED_Warning();//错误模式超过上限
                    GQ_Motor_send_current(0,0,0,0);
                }  
          }
    }
    else
    {
      hoisting_init();
      GQ_Motor_send_current(0,0,0,0);
    }

        vTaskDelay(2);
  }


}

/**
 * @brief 初始化云台控制结构体变量
 */
static void hoisting_init(void)
{
    for (int i = 1; i < 5; i++)
  {
    timed_return_motor_status(i, 5); 
    osDelay(1);
  }
  for (int i = 0; i < 4; i++)
  {
    // 初始化电机状态
    hoisting_control.hoisting_motor[i].relative_angle = GQ_Motor_Measure[i].position*2*PI;//这里是用弧度来标记位置
    hoisting_control.hoisting_motor[i].relative_angle_init = GQ_Motor_Measure[i].position*2*PI; 
    hoisting_control.hoisting_motor[i].relative_angle_set = GQ_Motor_Measure[i].position*2*PI;
    hoisting_control.hoisting_motor[i].motor_gyro = GQ_Motor_Measure[i].velocity*2*PI;//单位rad/s
  }
  //pid初始化
    PID_init(&hoisting_control.hoisting_motor[0].hoisting_motor_relative_pid,HOISTING_MOTOR_RELATIVE_KP_1,HOISTING_MOTOR_RELATIVE_KI_1,HOISTING_MOTOR_RELATIVE_KD_1,HOISTING_MOTOR_RELATIVE_MAX_OUT_1,HOISTING_MOTOR_RELATIVE_MAX_IOUT_1);
    PID_init(&hoisting_control.hoisting_motor[0].hoisting_motor_gyro_pid,HOISTING_MOTOR_GYRO_KP_1,HOISTING_MOTOR_GYRO_KI_1,HOISTING_MOTOR_GYRO_KD_1,HOISTING_MOTOR_GYRO_MAX_OUT_1,HOISTING_MOTOR_GYRO_MAX_IOUT_1);
    PID_init(&hoisting_control.hoisting_motor[1].hoisting_motor_relative_pid,HOISTING_MOTOR_RELATIVE_KP_2,HOISTING_MOTOR_RELATIVE_KI_2,HOISTING_MOTOR_RELATIVE_KD_2,HOISTING_MOTOR_RELATIVE_MAX_OUT_2,HOISTING_MOTOR_RELATIVE_MAX_IOUT_2);
    PID_init(&hoisting_control.hoisting_motor[1].hoisting_motor_gyro_pid,HOISTING_MOTOR_GYRO_KP_2,HOISTING_MOTOR_GYRO_KI_2,HOISTING_MOTOR_GYRO_KD_2,HOISTING_MOTOR_GYRO_MAX_OUT_2,HOISTING_MOTOR_GYRO_MAX_IOUT_2 );
    PID_init(&hoisting_control.hoisting_motor[2].hoisting_motor_relative_pid,HOISTING_MOTOR_RELATIVE_KP_3,HOISTING_MOTOR_RELATIVE_KI_3,HOISTING_MOTOR_RELATIVE_KD_3,HOISTING_MOTOR_RELATIVE_MAX_OUT_3,HOISTING_MOTOR_RELATIVE_MAX_IOUT_3);
    PID_init(&hoisting_control.hoisting_motor[2].hoisting_motor_gyro_pid,HOISTING_MOTOR_GYRO_KP_3, HOISTING_MOTOR_GYRO_KI_3 , HOISTING_MOTOR_GYRO_KD_3 , HOISTING_MOTOR_GYRO_MAX_OUT_3 , HOISTING_MOTOR_GYRO_MAX_IOUT_3);
    PID_init(&hoisting_control.hoisting_motor[3].hoisting_motor_relative_pid , HOISTING_MOTOR_RELATIVE_KP_4 , HOISTING_MOTOR_RELATIVE_KI_4 , HOISTING_MOTOR_RELATIVE_KD_4 , HOISTING_MOTOR_RELATIVE_MAX_OUT_4 , HOISTING_MOTOR_RELATIVE_MAX_IOUT_4 );
    PID_init(&hoisting_control.hoisting_motor[3].hoisting_motor_gyro_pid , HOISTING_MOTOR_GYRO_KP_4 , HOISTING_MOTOR_GYRO_KI_4 , HOISTING_MOTOR_GYRO_KD_4 , HOISTING_MOTOR_GYRO_MAX_OUT_4 , HOISTING_MOTOR_GYRO_MAX_IOUT_4);
}
static void hoisting_feedback_update(void)
{
  for (int i = 0; i < 4; i++)
  {
    hoisting_control.hoisting_motor[i].relative_angle = GQ_Motor_Measure[i].position*2*PI;
    hoisting_control.hoisting_motor[i].motor_gyro = GQ_Motor_Measure[i].velocity*2*PI;
  }
}

static void hoisting_control_loop(void)
{
  for (int i = 0; i < 4; i++)
  {
    hoisting_control.hoisting_motor[i].motor_gyro_set=PID_calc(&hoisting_control.hoisting_motor[i].hoisting_motor_relative_pid,hoisting_control.hoisting_motor[i].relative_angle,hoisting_control.hoisting_motor[i].relative_angle_set);
    hoisting_control.hoisting_motor[i].current_set=PID_calc(&hoisting_control.hoisting_motor[i].hoisting_motor_gyro_pid,hoisting_control.hoisting_motor[i].motor_gyro,hoisting_control.hoisting_motor[i].motor_gyro_set);
    hoisting_control.hoisting_motor[i].given_current=hoisting_control.hoisting_motor[i].current_set;
  }

}

static void hoisting_behavour_set(void)
{
  #ifdef DT7_rc_ctrl
    if (switch_is_down(rc_ctrl.rc.s[HOISTING_MODE_CHANNEL]))  
    {
        hoisting_control.hoisting_mode=HOISTING_MOTOR_GYRO;
    }
    else if (switch_is_mid(rc_ctrl.rc.s[HOISTING_MODE_CHANNEL])) // 中挡
    {
        hoisting_control.hoisting_mode=AUTO_HOISTING;
    }
    else if (switch_is_up(rc_ctrl.rc.s[HOISTING_MODE_CHANNEL]))  // 上挡
    {
        hoisting_control.hoisting_mode=SET_UP_HOISTING;
    }
  #endif

  #ifdef i6x_rc_ctrl
      // 遥控器设置模式（非必要不建议改）
    if (i6x_switch_is_down(i6x_ctrl.s[HOISTING_MODE_CHANNEL])) // 上挡
    {
      hoisting_control.hoisting_mode = SET_UP_HOISTING;
    }
    else if (i6x_switch_is_up(i6x_ctrl.s[HOISTING_MODE_CHANNEL])) // 下挡
    {
      hoisting_control.hoisting_mode = HOISTING_MOTOR_GYRO;
    }
  #endif

  #ifdef vtm_rc_ctrl
    if(vtm_rc_data.key & KEY_PRESSED_OFFSET_CTRL)//抬升
    {
      hoisting_control.hoisting_mode=AUTO_HOISTING;
    }
    else
    {
      hoisting_control.hoisting_mode = SET_UP_HOISTING;
    }
  #endif

}

static void hoisting_set_control(void)
{
  float forward_add=0;
  float backward_add=0;

switch(hoisting_control.hoisting_mode)
{
  case HOISTING_MOTOR_GYRO:
      hoisting_rc_to_speed(&forward_add, &backward_add);
      hoisting_control.hoisting_motor[0].relative_angle_set=hoisting_control.hoisting_motor[0].relative_angle_init+forward_angle_feedforward+forward_add;
      hoisting_control.hoisting_motor[1].relative_angle_set=hoisting_control.hoisting_motor[1].relative_angle_init-forward_angle_feedforward-forward_add;
      hoisting_control.hoisting_motor[2].relative_angle_set=hoisting_control.hoisting_motor[2].relative_angle_init+backward_angle_feedforward+backward_add;
      hoisting_control.hoisting_motor[3].relative_angle_set=hoisting_control.hoisting_motor[3].relative_angle_init-backward_angle_feedforward-backward_add;
    break;
  case AUTO_HOISTING:
     hoisting_control.hoisting_motor[0].relative_angle_set=hoisting_control.hoisting_motor[0].relative_angle_init-3.0f;
     hoisting_control.hoisting_motor[1].relative_angle_set=hoisting_control.hoisting_motor[1].relative_angle_init+3.0f;
     hoisting_control.hoisting_motor[2].relative_angle_set=hoisting_control.hoisting_motor[2].relative_angle_init-7.0f;
     hoisting_control.hoisting_motor[3].relative_angle_set=hoisting_control.hoisting_motor[3].relative_angle_init+7.0f;
    break;
  case SET_UP_HOISTING:
    hoisting_control.hoisting_motor[0].relative_angle_set=hoisting_control.hoisting_motor[0].relative_angle_init+forward_angle_feedforward;
    hoisting_control.hoisting_motor[1].relative_angle_set=hoisting_control.hoisting_motor[1].relative_angle_init-forward_angle_feedforward;
    hoisting_control.hoisting_motor[2].relative_angle_set=hoisting_control.hoisting_motor[2].relative_angle_init+backward_angle_feedforward;
    hoisting_control.hoisting_motor[3].relative_angle_set=hoisting_control.hoisting_motor[3].relative_angle_init-backward_angle_feedforward; 
    break;
}  

}

float add_forword=0;//累加值
float add_backward=0;
static void hoisting_rc_to_speed(float *forward, float *backward)
{

    static int16_t forward_channel = 0, back_channel = 0;

  #ifdef DT7_rc_ctrl
    rc_deadband_limit(rc_ctrl.rc.ch[FORWARD_CHANNEL], forward_channel, RC_TO_HOISTING_DEADBAND);
    rc_deadband_limit(rc_ctrl.rc.ch[BACK_CHANNEL], back_channel, RC_TO_HOISTING_DEADBAND);
  #endif

    add_forword += (float)forward_channel*forward_RC_SEN;
    add_backward += (float)back_channel*back_RC_SEN;
    *forward = add_forword;
    *backward = add_backward;

}

static void hoisting_update_offline_state(void)
{
  for (int i = 0; i < 4; i++)
  {
    hoisting_control.hoisting_motor[i].motor_gyro_set=0;
    hoisting_control.hoisting_motor[i].current_set=0;
    hoisting_control.hoisting_motor[i].given_current=0;
  }
}

 /**
   * @brief 堵转保护检测函数
   * @details 检测每个电机的位置误差，如果误差大于阈值且持续超过1秒，则进入堵转保护
   */
  static void hoisting_stall_protection(void)
  {
    uint32_t current_time = xTaskGetTickCount();  // 获取当前系统时间(ms)

    for (int i = 0; i < 4; i++)
    {
      // 计算位置误差
      float position_error = fabsf(hoisting_control.hoisting_motor[i].relative_angle_set -
                                    hoisting_control.hoisting_motor[i].relative_angle);

      // 判断位置误差是否超过阈值
      if (position_error > STALL_POSITION_THRESHOLD)
      {
        // 如果是首次检测到误差超限，记录开始时间
        if (stall_detect_time[i] == 0)
        {
          stall_detect_time[i] = current_time;
        }
        // 如果误差超限持续时间超过1秒，进入堵转保护
        else if ((current_time - stall_detect_time[i]) >= STALL_TIME_THRESHOLD)
        {
          stall_protection_flag[i] = 1;  // 设置堵转保护标志
          hoisting_control.hoisting_motor[i].given_current = 0;  // 清零电流输出
        }
      }
      else
      {
        // 位置误差正常，清除堵转检测
        stall_detect_time[i] = 0;
        stall_protection_flag[i] = 0;
      }
    }
  }

  /**
   * @brief 检查是否有电机处于堵转保护状态
   * @return 1-有电机堵转, 0-无堵转
   */
  static uint8_t hoisting_check_stall_state(void)
  {
    for (int i = 0; i < 4; i++)
    {
      if (stall_protection_flag[i] == 1)
      {
        return 1;  // 有电机处于堵转状态
      }
    }
    return 0;  // 所有电机正常
  }