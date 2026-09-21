#include "Hot_control.h"
#include "referee.h"
#include "shoot.h"
#ifdef HOT_CONTROL
           
     // 宏定义（按实际机构调整）
  #define HOT_HEAT_PER_SHOT    10.0f   // 17mm每发热量
  #define HOT_SHOOT_SPEED_MAX    10.0f   // 最大射速上限（发/s）
  #define HOT_SPEED_SCALE       275.0f   // 发/s
  #define HOT_SPEED_LIMIT      2200.0f   // 拨弹电机转速上限
  #define SHOOT_TIME_MIN        250U
  #define SHOOT_TIME_MAX       4000U

  static uint16_t shoot_time  = 0; 
  static uint16_t ShootTime   = 0;
  static float    shoot_speed = 0.0f;

  void hot_control(void)
  {
      uint16_t current_heat = referee_data.power_heat.shooter_17mm_1_barrel_heat;
      uint16_t heat_limit   = referee_data.robot.shooter_barrel_heat_limit*0.90f;
      uint16_t cooling_val  = referee_data.robot.shooter_barrel_cooling_value;
 
      
      float a = (float)cooling_val;
      float d = HOT_HEAT_PER_SHOT;
      float m = (heat_limit > current_heat) ? (float)(heat_limit - current_heat) : 0.0f;

      // 剩余热量不足一发，立即停止
      if (m < d)
      {
          shoot_cmd_send.trigger_speed = 0.0f;
          shoot_time = 0;
          return;
      }

      // 阶段一：每轮起始计算连发参数
      if (shoot_time == 0)
      {
          float t = (m + 2.0f * a) * 10.0f;
          ShootTime = (t < SHOOT_TIME_MIN) ? SHOOT_TIME_MIN
                    : (t > SHOOT_TIME_MAX) ? SHOOT_TIME_MAX
                    : (uint16_t)t;

          float denom = d * ((float)ShootTime / 100.0f);
          if (m < 200.0f)
              shoot_speed = (10.0f * m - a - 3.0f * d) / denom + a / d;
          else
              shoot_speed = (10.0f * m - a - 5.0f * d) / denom + a / d;

          if (shoot_speed < 0.0f)                shoot_speed = 0.0f;
          if (shoot_speed > HOT_SHOOT_SPEED_MAX) shoot_speed = HOT_SHOOT_SPEED_MAX;
      }

      // 阶段二：连发阶段
      if (shoot_time < ShootTime)
      {
          shoot_cmd_send.trigger_speed = shoot_speed * HOT_SPEED_SCALE;
          if (shoot_cmd_send.trigger_speed > HOT_SPEED_LIMIT)
              shoot_cmd_send.trigger_speed = HOT_SPEED_LIMIT;
          shoot_time++;
      }
      // 阶段三：冷却等待阶段
      else
      {
          shoot_cmd_send.trigger_speed = (a / d) * HOT_SPEED_SCALE;
          if (shoot_cmd_send.trigger_speed > HOT_SPEED_LIMIT)
              shoot_cmd_send.trigger_speed = HOT_SPEED_LIMIT;

          // 热量冷却恢复后自动重置，触发下一轮
          if (m >= d * 2.0f)
              shoot_time = 0;
      }
  }


#endif

