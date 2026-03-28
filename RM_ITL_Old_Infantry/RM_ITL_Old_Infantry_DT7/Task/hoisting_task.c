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

uint16_t error_count = 0;
hoisting_t hoisting_control;
float add_forward=0;//累加值
float add_backward=0;

static void hoisting_init(void);
static void hoisting_feedback_update(void);
static void hoisting_control_loop(void);
static void hoisting_behavour_set(void);
static void hoisting_set_control(void);
static void hoisting_rc_to_speed(float *forward, float *backward);
static void hoisting_update_offline_state(void);
static void hoisting_limit_control(void);
static void hoisting_wake_up(void);
static void protect_hoisting_motor(void);

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

    hoisting_limit_control();

    protect_hoisting_motor();

    if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && 
          toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
    {
          if (toe_is_error(DBUS_TOE))  // 遥控器离线
          {
            hoisting_update_offline_state();
            GQ_Motor_send_current(0,0,0,0);
            hoisting_wake_up();
            osDelay(2);
          }
          else
          {
                if (fabsf(GQ_Motor_Measure[0].position) < 3 && fabsf(GQ_Motor_Measure[1].position) < 3 
                    && fabsf(GQ_Motor_Measure[2].position) < 3 && fabsf(GQ_Motor_Measure[3].position) < 3)
                {
                  //GQ_Motor_send_current(0,0,0,0);
                    GQ_Motor_send_current(
                    hoisting_control.hoisting_motor[0].given_current,hoisting_control.hoisting_motor[1].given_current,
                    hoisting_control.hoisting_motor[2].given_current,hoisting_control.hoisting_motor[3].given_current);
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
      error_count++;  
    }
    if (error_count > 100 )
    {
      GQ_Motor_send_current(0,0,0,0);
      hoisting_wake_up();
      osDelay(2);
      error_count=0;
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

    add_forward=0.0f;
    add_backward=0.0f;
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
      hoisting_control.hoisting_mode = AUTO_HOISTING;
    }
    else if (i6x_switch_is_up(i6x_ctrl.s[HOISTING_MODE_CHANNEL])) // 下挡
    {
      hoisting_control.hoisting_mode = SET_UP_HOISTING;
    }
  #endif

  #ifdef vtm_rc_ctrl
    if(vtm_rc_data.mouse_middle)//抬升
    {
      hoisting_control.hoisting_mode=AUTO_HOISTING;
    }
    else
    {
      hoisting_control.hoisting_mode = SET_UP_HOISTING;
    }

    if(vtm_rc_data.mode_sw == 1 || vtm_rc_data.mode_sw == 2)
    {
      if(vtm_rc_data.pause)//下降
      {
        hoisting_control.hoisting_mode=AUTO_HOISTING;
      }
      else
      {
        hoisting_control.hoisting_mode = SET_UP_HOISTING;
      }
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
      hoisting_control.hoisting_motor[0].relative_angle_set=hoisting_control.hoisting_motor[0].relative_angle_init-HOISTING_AUTO_FORWARD_ANGLE_OFFSET;
      hoisting_control.hoisting_motor[1].relative_angle_set=hoisting_control.hoisting_motor[1].relative_angle_init+HOISTING_AUTO_FORWARD_ANGLE_OFFSET;
      hoisting_control.hoisting_motor[2].relative_angle_set=hoisting_control.hoisting_motor[2].relative_angle_init-HOISTING_AUTO_BACKWARD_ANGLE_OFFSET;
      hoisting_control.hoisting_motor[3].relative_angle_set=hoisting_control.hoisting_motor[3].relative_angle_init+HOISTING_AUTO_BACKWARD_ANGLE_OFFSET;
    break;
  case SET_UP_HOISTING:
      hoisting_control.hoisting_motor[0].relative_angle_set=hoisting_control.hoisting_motor[0].relative_angle_init+forward_angle_feedforward;
      hoisting_control.hoisting_motor[1].relative_angle_set=hoisting_control.hoisting_motor[1].relative_angle_init-forward_angle_feedforward;
      hoisting_control.hoisting_motor[2].relative_angle_set=hoisting_control.hoisting_motor[2].relative_angle_init+backward_angle_feedforward;
      hoisting_control.hoisting_motor[3].relative_angle_set=hoisting_control.hoisting_motor[3].relative_angle_init-backward_angle_feedforward; 
      break;
}  

}


static void hoisting_rc_to_speed(float *forward, float *backward)
{

    static int16_t forward_channel = 0, back_channel = 0;

  #ifdef DT7_rc_ctrl
    rc_deadband_limit(rc_ctrl.rc.ch[FORWARD_CHANNEL], forward_channel, RC_TO_HOISTING_DEADBAND);
    rc_deadband_limit(rc_ctrl.rc.ch[BACK_CHANNEL], back_channel, RC_TO_HOISTING_DEADBAND);
  #endif

    add_forward += (float)forward_channel*forward_RC_SEN;
    add_backward += (float)back_channel*back_RC_SEN;
    *forward = add_forward;
    *backward = add_backward;

}

static void hoisting_update_offline_state(void)
{
  for (int i = 0; i < 4; i++)
  {
    hoisting_control.hoisting_motor[i].current_set=0;
    hoisting_control.hoisting_motor[i].given_current=0;
    add_forward=0.0f;
    add_backward=0.0f;
  }
}

static void hoisting_limit_control(void)
{
  if(limit_switch.hoisting_1)
  {
    // 初始化电机状态
    hoisting_control.hoisting_motor[0].relative_angle = GQ_Motor_Measure[0].position*2*PI;//这里是用弧度来标记位置
    hoisting_control.hoisting_motor[0].relative_angle_init = GQ_Motor_Measure[0].position*2*PI; 
    hoisting_control.hoisting_motor[0].relative_angle_set = GQ_Motor_Measure[0].position*2*PI;
    hoisting_control.hoisting_motor[0].motor_gyro = GQ_Motor_Measure[0].velocity*2*PI;//单位rad/s
    add_forward=0.0f;
    add_backward=0.0f;
  }
  if(limit_switch.hoisting_2)
  {
    // 初始化电机状态
    hoisting_control.hoisting_motor[1].relative_angle = GQ_Motor_Measure[1].position*2*PI;//这里是用弧度来标记位置
    hoisting_control.hoisting_motor[1].relative_angle_init = GQ_Motor_Measure[1].position*2*PI; 
    hoisting_control.hoisting_motor[1].relative_angle_set = GQ_Motor_Measure[1].position*2*PI;
    hoisting_control.hoisting_motor[1].motor_gyro = GQ_Motor_Measure[1].velocity*2*PI;//单位rad/s
    add_forward=0.0f;
    add_backward=0.0f;
  }
  if(limit_switch.hoisting_3)
  {
    // 初始化电机状态
    hoisting_control.hoisting_motor[2].relative_angle = GQ_Motor_Measure[2].position*2*PI;//这里是用弧度来标记位置
    hoisting_control.hoisting_motor[2].relative_angle_init = GQ_Motor_Measure[2].position*2*PI; 
    hoisting_control.hoisting_motor[2].relative_angle_set = GQ_Motor_Measure[2].position*2*PI;
    hoisting_control.hoisting_motor[2].motor_gyro = GQ_Motor_Measure[2].velocity*2*PI;//单位rad/s
    add_forward=0.0f;
    add_backward=0.0f;
  }
  if(limit_switch.hoisting_4)
  {
    // 初始化电机状态
    hoisting_control.hoisting_motor[3].relative_angle = GQ_Motor_Measure[3].position*2*PI;//这里是用弧度来标记位置
    hoisting_control.hoisting_motor[3].relative_angle_init = GQ_Motor_Measure[3].position*2*PI; 
    hoisting_control.hoisting_motor[3].relative_angle_set = GQ_Motor_Measure[3].position*2*PI;
    hoisting_control.hoisting_motor[3].motor_gyro = GQ_Motor_Measure[3].velocity*2*PI;//单位rad/s
    add_forward=0.0f;
    add_backward=0.0f;
  }

}

static void hoisting_wake_up(void)
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
      hoisting_control.hoisting_motor[i].relative_angle_set = GQ_Motor_Measure[i].position*2*PI;
      hoisting_control.hoisting_motor[i].motor_gyro = GQ_Motor_Measure[i].velocity*2*PI;//单位rad/s
    }
    hoisting_control.hoisting_motor[0].relative_angle_init = GQ_Motor_Measure[0].position*2*PI-forward_angle_feedforward;
    hoisting_control.hoisting_motor[1].relative_angle_init = GQ_Motor_Measure[1].position*2*PI+forward_angle_feedforward;
    hoisting_control.hoisting_motor[2].relative_angle_init = GQ_Motor_Measure[2].position*2*PI-backward_angle_feedforward; 
    hoisting_control.hoisting_motor[3].relative_angle_init = GQ_Motor_Measure[3].position*2*PI+backward_angle_feedforward;
    add_forward=0.0f;
    add_backward=0.0f;

}

static void protect_hoisting_motor(void)
{
  if(vtm_rc_data.key & KEY_PRESSED_OFFSET_V)
  {
    while (1)
    {
      GQ_Motor_send_current(0,0,0,0);
      vTaskDelay(2);
    }
    

  }

}
