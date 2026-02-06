 /*
***********************某虽不才，即断对面之头，悬之东门，以谢天下***********************

*
                        1 1 1 1 1 1 1 1 1 1                 1
                                          1                 1
                                          1                 1
                                          1                 1
                                          1                 1
                                          1                 1
                                          1                 1
                                          1                 1
                        1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 
                        1                 1
                        1                 1
                        1                 1
                        1                 1
                        1                 1
                        1                 1
                        1                 1
                        1                 1 1 1 1 1 1 1 1 1 1 
												
												
***********************某虽不才，即断对面之头，悬之东门，以谢天下***********************

*/
#include "shoot_task.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "user_lib.h"
#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "usart.h"
#include "USART_receive.h"

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
static void shoot_init(void);

/**
 * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void);

/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void);

/**
 * @brief          射击循环
 * @param[in]      void
 * @retval         返回can控制值
 */
static void shoot_control_loop(void);

static void detect_bullet_count(void);

/* =========================== 变量声明 =========================== */
#ifdef vtm_rc_ctrl
    uint8_t shoot_mode_index = 0;
#endif

//射击数据
shoot_control_t shoot_control;
//发送的电机电流

static int16_t left_friction_can_set_current = 0, right_friction_can_set_current = 0, trigger_can_set_current = 0;

void shoot_task(void const *pvParameters)
{
  //空闲一段时间
  vTaskDelay(SHOOT_TASK_INIT_TIME);

  //发射机构初始化
  shoot_init();


  while (1)
  {
    //射击模式选择
    shoot_set_mode();
    //发射机构数据更新
    shoot_feedback_update();
    //射击控制量计算
    shoot_control_loop();


    left_friction_can_set_current = shoot_control.left_fricition_motor.given_current;
    right_friction_can_set_current = shoot_control.right_fricition_motor.given_current;
    trigger_can_set_current = shoot_control.trigger_motor.given_current;
    
        
    if (!(toe_is_error(FRONT_LEFT_FRICTION_MOTOR_TOE) || toe_is_error(FRONT_RIGHT_FRICTION_MOTOR_TOE)))
    {
        if (toe_is_error(DBUS_TOE))
        {
            CAN_cmd_friction(0, 0);
            CAN_cmd_shoot(0);
            shoot_control.trigger_motor.set_angle = shoot_control.trigger_motor.angle;
        }
        else
        {
            CAN_cmd_friction(left_friction_can_set_current, right_friction_can_set_current);
            CAN_cmd_shoot(trigger_can_set_current);
        }
    }
    else
    {
            CAN_cmd_friction(0, 0);
            CAN_cmd_shoot(0);
            shoot_control.trigger_motor.set_angle = shoot_control.trigger_motor.angle;
    }
    
    vTaskDelay(SHOOT_CONTROL_TIME);
  }
}

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */





void shoot_init(void)
{
  shoot_control.shoot_mode =SHOOT_STOP;
  //电机指针
  shoot_control.trigger_motor.motor_measure = get_trigger_motor_measure_point();
  shoot_control.left_fricition_motor.motor_measure = get_left_friction_motor_measure_point();
  shoot_control.right_fricition_motor.motor_measure = get_right_friction_motor_measure_point();
  //初始化PID
  PID_init(&shoot_control.trigger_motor.motor_speed_pid, TRIGGER_KP, TRIGGER_KI, TRIGGER_KD, TRIGGER_MAX_OUT, TRIGGER_MAX_IOUT); //拨弹电机

  PID_init(&shoot_control.left_fricition_motor.motor_speed_pid, FRICTION_KP, FRICTION_KI, FRICTION_KD, FRICTION_MAX_OUT, FRICTION_MAX_IOUT); //左摩擦轮

  PID_init(&shoot_control.right_fricition_motor.motor_speed_pid, FRICTION_KP, FRICTION_KI, FRICTION_KD, FRICTION_MAX_OUT, FRICTION_MAX_IOUT); //右摩擦轮
  //更新数据
  shoot_feedback_update();
  //拨弹电机
  shoot_control.trigger_motor.ecd_count = 0;
  shoot_control.trigger_motor.angle = shoot_control.trigger_motor.motor_measure->angle;
  shoot_control.trigger_motor.given_current = 0;
  shoot_control.trigger_motor.set_angle = shoot_control.trigger_motor.angle;
  shoot_control.trigger_motor.speed = 0.0f;
  shoot_control.trigger_motor.speed_set = 0.0f;
  //左摩擦轮
  shoot_control.left_fricition_motor.accel = 0.0f;
  shoot_control.left_fricition_motor.speed = 0.0f;
  shoot_control.left_fricition_motor.speed_set = 0.0f;
  shoot_control.left_fricition_motor.given_current = 0;
  //右摩擦轮
  shoot_control.right_fricition_motor.accel = 0.0f;
  shoot_control.right_fricition_motor.speed = 0.0f;
  shoot_control.right_fricition_motor.speed_set = 0.0f;
  shoot_control.right_fricition_motor.given_current = 0;

  shoot_control.shoot_count=0;
	
}

/**
 * @brief          射击状态机设置，遥控器上拨1次发射数颗，一直处在上，则持续发射，用于3min准备时间清理子弹
 * @param[in]      void
 * @retval         void
 */

static void shoot_set_mode(void)
 {

  #ifdef DT7_rc_ctrl 
    //处于上档，进入发射模式
		if (switch_is_up(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
			shoot_control.shoot_mode = SHOOT_BULLET;			
    }
  //处于中档，开启摩擦轮
    else if (switch_is_mid(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
  //处于下档，关闭摩擦轮
		else if (switch_is_down(rc_ctrl.rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
    else
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
  #endif
  
  #ifdef i6x_rc_ctrl

  //处于下档，开启摩擦轮
    if (i6x_switch_is_down(i6x_ctrl.s[FRICTION_RC_MODE_CHANNEL]))
    {
        shoot_control.shoot_mode = SHOOT_READY; 
    }
  //处于上档，关闭摩擦轮
		else if (i6x_switch_is_up(i6x_ctrl.s[FRICTION_RC_MODE_CHANNEL]))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
		if (i6x_switch_is_down(i6x_ctrl.s[SHOOT_RC_MODE_CHANNEL]))
    {
			shoot_control.shoot_mode = SHOOT_BULLET;			
    }
  		else if (i6x_switch_is_up(i6x_ctrl.s[SHOOT_RC_MODE_CHANNEL]))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
  #endif

  #ifdef vtm_rc_ctrl
        // 检测R键是否按下（上升沿触发）
        static uint8_t r_key_last_state = 0;
        uint8_t r_key_current_state = (vtm_rc_data.key & KEY_PRESSED_OFFSET_F) ? 1 : 0;

        // 上升沿检测：当前按下且上次未按下
        if (r_key_current_state && !r_key_last_state)
        {
            // 切换到下一个模式（循环切换）
            shoot_mode_index = (shoot_mode_index + 1) % 2; // 2种模式

            // 根据索引设置底盘模式
            switch (shoot_mode_index)
            {
                case 0:
                    shoot_control.shoot_mode = SHOOT_STOP;
                    break;
                case 1:
                    shoot_control.shoot_mode = SHOOT_READY; 
                    break;
            }
        }

        // 更新上一次R键状态
        r_key_last_state = r_key_current_state;

        if(shoot_control.shoot_mode == SHOOT_READY)
        {
          //手动开火
          if (open_fire(vtm_rc_data.mouse_left))
          {
            shoot_control.shoot_mode = SHOOT_BULLET;
          }
          else if (hold_fire(vtm_rc_data.mouse_left))
          {
            shoot_control.shoot_mode = SHOOT_READY;
          }
          //自动开火
          if (open_fire(vtm_rc_data.mouse_right))
          {
            if(auto_shoot.mode == 2)//把开火权交给自瞄
            {
              shoot_control.shoot_mode = SHOOT_BULLET;
            }            
          }
          else if (hold_fire(vtm_rc_data.mouse_right))
          {
            shoot_control.shoot_mode = SHOOT_READY;
          }
        }
  #endif
    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
}




/**
 * @brief 射击数据更新
 *@param[in] void
 *@retval void
 */
static void shoot_feedback_update(void)
{

  //拨弹轮
  shoot_control.trigger_motor.speed = shoot_control.trigger_motor.motor_measure->speed_rpm;
   //上次左摩擦轮转速
  shoot_control.left_fricition_motor.last_speed = shoot_control.left_fricition_motor.speed; 
  //上次右摩擦轮转速
  shoot_control.right_fricition_motor.last_speed = shoot_control.right_fricition_motor.speed;
  //当前左摩擦轮转速
  shoot_control.left_fricition_motor.speed = shoot_control.left_fricition_motor.motor_measure->speed_rpm;
  //当前右摩擦轮转速
  shoot_control.right_fricition_motor.speed = shoot_control.right_fricition_motor.motor_measure->speed_rpm;

  detect_bullet_count();//检测发射计数
}

/**
 * @brief          射击循环
 * @param[in]      void
 * @retval         返回can控制值
 */
static void shoot_control_loop(void)
{
  if (shoot_control.shoot_mode == SHOOT_STOP)
  {
    //设置拨弹轮的速度
    shoot_control.trigger_motor.speed_set = 0.0f;
    shoot_control.left_fricition_motor.speed_set = 0.0f;
    shoot_control.right_fricition_motor.speed_set = 0.0f;

    //设置加速减速电流限制
    shoot_control.left_fricition_motor.motor_speed_pid.max_out = FRICTION_ACCEL_MAX_OUT;
    shoot_control.right_fricition_motor.motor_speed_pid.max_out = FRICTION_ACCEL_MAX_OUT;
  }
  else if (shoot_control.shoot_mode == SHOOT_READY)
  {
    //设置拨弹轮的速度
    shoot_control.trigger_motor.speed_set = 0.0f;
    //摩擦轮缓启动
    if (shoot_control.right_fricition_motor.speed > (FRICTION_SPEED_SET * 0.0f))
    {
      shoot_control.left_fricition_motor.motor_speed_pid.max_out = FRICTION_MAX_OUT;
      shoot_control.right_fricition_motor.motor_speed_pid.max_out = FRICTION_MAX_OUT;
    }
	  shoot_control.left_fricition_motor.speed_set = -FRICTION_SPEED_SET;
    shoot_control.right_fricition_motor.speed_set = FRICTION_SPEED_SET;
  }
  else if (shoot_control.shoot_mode == SHOOT_BULLET)
  {
		shoot_control.trigger_motor.speed_set = CONTINUE_TRIGGER_SPEED;		
  }

  //计算拨弹轮电机PID
	
	shoot_control.trigger_motor.angle=shoot_control.trigger_motor.motor_measure->angle;
	
	PID_calc(&shoot_control.trigger_motor.motor_speed_pid, shoot_control.trigger_motor.speed,shoot_control.trigger_motor.speed_set);
  
  shoot_control.trigger_motor.given_current = (int16_t)(shoot_control.trigger_motor.motor_speed_pid.out);
	
	

  //计算摩擦轮电机PID
  PID_calc(&shoot_control.left_fricition_motor.motor_speed_pid, shoot_control.left_fricition_motor.speed, shoot_control.left_fricition_motor.speed_set);
  shoot_control.left_fricition_motor.given_current = (int16_t)(shoot_control.left_fricition_motor.motor_speed_pid.out);

  PID_calc(&shoot_control.right_fricition_motor.motor_speed_pid, shoot_control.right_fricition_motor.speed, shoot_control.right_fricition_motor.speed_set);
  shoot_control.right_fricition_motor.given_current = (int16_t)(shoot_control.right_fricition_motor.motor_speed_pid.out);
	
}

static void detect_bullet_count(void)
{
  if (fabsf(shoot_control.left_fricition_motor.speed - shoot_control.left_fricition_motor.last_speed) > SPEED_WAVE_THRESHOLD && 
      fabsf(shoot_control.right_fricition_motor.speed - shoot_control.right_fricition_motor.last_speed) > SPEED_WAVE_THRESHOLD)
  {
    shoot_control.shoot_count++;
  }

}
