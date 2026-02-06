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

#define PI 3.14159265358979323846f
hoisting_motor_t hoisting_motor[4];

static void hoisting_init(void);
static void hoisting_feedback_update(void);
static void hoisting_control_loop(void);
static void hoisting_behavour_set(void);
static void hoisting_set_control(void);
static void hoisting_rc_to_speed(float *forward, float *backward);

void hoisting_task(void const *pvParameters)
{
  //空闲一段时间
  vTaskDelay(HOISTING_TASK_INIT_TIME);

  hoisting_init();
  Upper_Computer_Init(&hoisting_motor[1].relative_angle);
  Upper_Computer_Init(&hoisting_motor[1].relative_angle_set);
  while(1)
  {

    usart_vofa_send(&huart7);
    hoisting_behavour_set();

    hoisting_feedback_update();
    hoisting_set_control();

    hoisting_control_loop();
      if (toe_is_error(DBUS_TOE))  // 遥控器离线
      {
        LED_RC_Disconnected();//遥控离线模式
        GQ_Motor_send_current(0,0,0,0);
      }
      else
      {	
        if (fabsf(GQ_Motor_Measure[0].position) < 3 && fabsf(GQ_Motor_Measure[1].position) < 3 
            && fabsf(GQ_Motor_Measure[2].position) < 3 && fabsf(GQ_Motor_Measure[3].position) < 3)
        {
            LED_Normal();//正常启动
            GQ_Motor_send_current(hoisting_motor[0].given_current,hoisting_motor[1].given_current,hoisting_motor[2].given_current,hoisting_motor[3].given_current);
        } 
        else
        {
            LED_Warning();//错误模式超过上限
            GQ_Motor_send_current(0,0,0,0);
        }  
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
    hoisting_motor[i].relative_angle = GQ_Motor_Measure[i].position*2*PI;//这里是用弧度来标记位置
    hoisting_motor[i].relative_angle_init = GQ_Motor_Measure[i].position*2*PI; 
    hoisting_motor[i].relative_angle_set = GQ_Motor_Measure[i].position*2*PI;
    hoisting_motor[i].motor_gyro = GQ_Motor_Measure[i].velocity*2*PI;//单位rad/s
  }
  //pid初始化
    PID_init(&hoisting_motor[0].hoisting_motor_relative_pid,HOISTING_MOTOR_RELATIVE_KP_1,HOISTING_MOTOR_RELATIVE_KI_1,HOISTING_MOTOR_RELATIVE_KD_1,HOISTING_MOTOR_RELATIVE_MAX_OUT_1,HOISTING_MOTOR_RELATIVE_MAX_IOUT_1);
    PID_init(&hoisting_motor[0].hoisting_motor_gyro_pid,HOISTING_MOTOR_GYRO_KP_1,HOISTING_MOTOR_GYRO_KI_1,HOISTING_MOTOR_GYRO_KD_1,HOISTING_MOTOR_GYRO_MAX_OUT_1,HOISTING_MOTOR_GYRO_MAX_IOUT_1);
    PID_init(&hoisting_motor[1].hoisting_motor_relative_pid,HOISTING_MOTOR_RELATIVE_KP_2,HOISTING_MOTOR_RELATIVE_KI_2,HOISTING_MOTOR_RELATIVE_KD_2,HOISTING_MOTOR_RELATIVE_MAX_OUT_2,HOISTING_MOTOR_RELATIVE_MAX_IOUT_2);
    PID_init(&hoisting_motor[1].hoisting_motor_gyro_pid,HOISTING_MOTOR_GYRO_KP_2,HOISTING_MOTOR_GYRO_KI_2,HOISTING_MOTOR_GYRO_KD_2,HOISTING_MOTOR_GYRO_MAX_OUT_2,HOISTING_MOTOR_GYRO_MAX_IOUT_2 );
    PID_init(&hoisting_motor[2].hoisting_motor_relative_pid,HOISTING_MOTOR_RELATIVE_KP_3,HOISTING_MOTOR_RELATIVE_KI_3,HOISTING_MOTOR_RELATIVE_KD_3,HOISTING_MOTOR_RELATIVE_MAX_OUT_3,HOISTING_MOTOR_RELATIVE_MAX_IOUT_3);
    PID_init(&hoisting_motor[2].hoisting_motor_gyro_pid,HOISTING_MOTOR_GYRO_KP_3, HOISTING_MOTOR_GYRO_KI_3 , HOISTING_MOTOR_GYRO_KD_3 , HOISTING_MOTOR_GYRO_MAX_OUT_3 , HOISTING_MOTOR_GYRO_MAX_IOUT_3);
    PID_init(&hoisting_motor[3].hoisting_motor_relative_pid , HOISTING_MOTOR_RELATIVE_KP_4 , HOISTING_MOTOR_RELATIVE_KI_4 , HOISTING_MOTOR_RELATIVE_KD_4 , HOISTING_MOTOR_RELATIVE_MAX_OUT_4 , HOISTING_MOTOR_RELATIVE_MAX_IOUT_4 );
    PID_init(&hoisting_motor[3].hoisting_motor_gyro_pid , HOISTING_MOTOR_GYRO_KP_4 , HOISTING_MOTOR_GYRO_KI_4 , HOISTING_MOTOR_GYRO_KD_4 , HOISTING_MOTOR_GYRO_MAX_OUT_4 , HOISTING_MOTOR_GYRO_MAX_IOUT_4 );
}  


static void hoisting_feedback_update(void)
{
  for (int i = 0; i < 4; i++)
  {
    hoisting_motor[i].relative_angle = GQ_Motor_Measure[i].position*2*PI;
    hoisting_motor[i].motor_gyro = GQ_Motor_Measure[i].velocity*2*PI;
  }
}

static void hoisting_control_loop(void)
{
  for (int i = 0; i < 4; i++)
  {
    hoisting_motor[i].motor_gyro_set=PID_calc(&hoisting_motor[i].hoisting_motor_relative_pid,hoisting_motor[i].relative_angle,hoisting_motor[i].relative_angle_set);
    hoisting_motor[i].current_set=PID_calc(&hoisting_motor[i].hoisting_motor_gyro_pid,hoisting_motor[i].motor_gyro,hoisting_motor[i].motor_gyro_set);
    hoisting_motor[i].given_current=hoisting_motor[i].current_set;
  }

}

static void hoisting_behavour_set(void)
{

    if (switch_is_down(rc_ctrl.rc.s[LEFT_SWITCH]))  
    {
        hoisting_motor[0].hoisting_motor_mode=HOISTING_MOTOR_GYRO;
        hoisting_motor[1].hoisting_motor_mode=HOISTING_MOTOR_GYRO;
        hoisting_motor[2].hoisting_motor_mode=HOISTING_MOTOR_GYRO;
        hoisting_motor[3].hoisting_motor_mode=HOISTING_MOTOR_GYRO;
    }
    else
    {
        hoisting_motor[0].hoisting_motor_mode=HOISTING_MOTOR_RAW;
        hoisting_motor[1].hoisting_motor_mode=HOISTING_MOTOR_RAW;
        hoisting_motor[2].hoisting_motor_mode=HOISTING_MOTOR_RAW;
        hoisting_motor[3].hoisting_motor_mode=HOISTING_MOTOR_RAW;
    }
}

static void hoisting_set_control(void)
{
  float forward_add=0;
  float backward_add=0;
  hoisting_rc_to_speed(&forward_add, &backward_add);
  if (switch_is_down(rc_ctrl.rc.s[LEFT_SWITCH]))  // 下
  {
      hoisting_motor[0].relative_angle_set=hoisting_motor[0].relative_angle_init+forward_add;
      hoisting_motor[1].relative_angle_set=hoisting_motor[1].relative_angle_init+forward_add;
      hoisting_motor[2].relative_angle_set=hoisting_motor[2].relative_angle_init+backward_add;
      hoisting_motor[3].relative_angle_set=hoisting_motor[3].relative_angle_init+backward_add;
  }

}

float add_forword=0;//累加值
float add_backward=0;
static void hoisting_rc_to_speed(float *forward, float *backward)
{

    static int16_t forward_channel = 0, back_channel = 0;

    rc_deadband_limit(i6x_ctrl.ch[FORWARD_CHANNEL], forward_channel, RC_TO_HOISTING_DEADBAND);
    rc_deadband_limit(i6x_ctrl.ch[BACK_CHANNEL], back_channel, RC_TO_HOISTING_DEADBAND);

    add_forword += (float)forward_channel*forward_RC_SEN;
    add_backward += (float)back_channel*back_RC_SEN;
    *forward = add_forword;
    *backward = add_backward;

}