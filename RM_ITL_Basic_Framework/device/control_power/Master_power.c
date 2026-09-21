#include "Master_power.h"
#include "Lowpass.h"
#include "referee.h"
#include "remote_control.h"
#include <math.h>
#ifdef COMPILE_M_POWER
  // ===================== 内部状态 =====================
  static Manager     manager;
  static PowerStatus powerStatus;
  static uint16_t    motorDisconnectCounter[4] = {0U, 0U, 0U, 0U};
  static uint8_t     isInitialized = 0;
  static float       boost = 90.0f;

  // 功率能量环 PID
  static PID powerPD_full;

  // ===================== 内部工具函数 =====================
  static inline float float_equal(float a, float b) { return fabsf(a - b) < 1e-5f; }

  static inline void   set_flag(uint8_t *f, ErrorFlags flag)   { *f |=  (uint8_t)flag; }
  static inline void clear_flag(uint8_t *f, ErrorFlags flag)   { *f &= ~(uint8_t)flag; }
  static inline uint8_t is_flagged(uint8_t *f, ErrorFlags flag){ return (*f & (uint8_t)flag) != 0; }

  static inline float rpm2av(float rpm) { return rpm * RPM_TO_RADS; }

  static inline uint8_t isMotorConnected(int i)
  {
      return (manager.motors[i] != NULL) && (motorDisconnectCounter[i] < 10U);
  }

  static inline uint8_t isAllMotorConnected(void)
  {
      for (int i = 0; i < 4; i++)
          if (!isMotorConnected(i)) return 0;
      return 1;
  }

  float get_referee_chassis_power_limit(void)
  {
//       return (float)referee_data.robot.chassis_power_limit; //
    return boost;
  }

  float get_referee_buffer_energy(void)
  {
      // TODO: 接入裁判系统后改为返回 referee_data.power_heat.buffer_energy
      return 60.0f;
  }

  // ===================== 公开接口 =====================
  float getLatestFeedbackJudgePowerLimit(void)
  {
      return manager.refereeMaxPower;
  }

  const volatile PowerStatus *getPowerStatus(void)
  {
      return &powerStatus;
  }

  void MasterPower_SetRLSEnable(uint8_t enable)
  {
      manager.rls_enable = enable;
  }

  // ===================== 功率分配核心 =====================
  float *getControlledOutput(PowerObj *objs[4])
  {
      const float k0 = manager.torqueConst * MOTOR_MAX_CURRENT / MOTOR_MAX_OUTPUT;

      static float newTorqueCurrent[4];

      float sumCmdPower = 0.0f;
      float cmdPower[4] = {0};
      float sumError    = 0.0f;
      float error[4]    = {0};
      float maxPower    = manager.fullMaxPower;
      float allocatablePower = maxPower;
      float sumPowerRequired = 0.0f;

      for (int i = 0; i < 4; i++)
      {
          if (isMotorConnected(i))
          {
              PowerObj *p = objs[i];

              float torque = p->pidOutput * k0;
              cmdPower[i] = torque * p->curAv
                          + fabsf(p->curAv)      * manager.k1
                          + torque * torque       * manager.k2
                          + manager.k3 / 4.0f;

              sumCmdPower += cmdPower[i];
              error[i] = fabsf(p->setAv - p->curAv);
              if (float_equal(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f)
                  allocatablePower += -cmdPower[i];
              else
              {
                  sumError        += error[i];
                  sumPowerRequired += cmdPower[i];
              }
          }
          else if (motorDisconnectCounter[i] < 1000U)
          {
              float rpm  = manager.motors[i] ? manager.motors[i]->vel : 0.0f;
              float cur  = manager.motors[i] ? manager.motors[i]->current * k0 : 0.0f;
              float omega = rpm2av(rpm);
              cmdPower[i] = cur * omega
                          + fabsf(omega) * manager.k1
                          + cur * cur    * manager.k2
                          + manager.k3 / 4.0f;

              error[i] = 0.0f;
          }
          else
          {
              cmdPower[i] = 0.0f;
              error[i]    = 0.0f;
          }
      }

      powerStatus.maxPowerLimited          = maxPower;
      powerStatus.sumPowerCmd_before_clamp = sumCmdPower;

      if (sumCmdPower > maxPower)
      {
          float errorConfidence;
          if (sumError > ERROR_POWER_DISTRIBUTION_SET)
              errorConfidence = 1.0f;
          else if (sumError > PROP_POWER_DISTRIBUTION_SET)
              errorConfidence = _constrain(
                  (sumError - PROP_POWER_DISTRIBUTION_SET)
                  / (ERROR_POWER_DISTRIBUTION_SET - PROP_POWER_DISTRIBUTION_SET),
                  0.0f, 1.0f);
          else
              errorConfidence = 0.0f;

          for (int i = 0; i < 4; i++)
          {
              PowerObj *p = objs[i];
              if (!isMotorConnected(i)) { newTorqueCurrent[i] = 0.0f; continue; }

              if (float_equal(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f)
              {
                  newTorqueCurrent[i] = p->pidOutput;
                  continue;
              }
              float wErr  = (sumError > 1e-5f) ? fabsf(p->setAv - p->curAv) / sumError : 0.0f;
              float wProp = (sumPowerRequired > 1e-5f) ? cmdPower[i] / sumPowerRequired : 0.0f;
              float wt    = errorConfidence * wErr + (1.0f - errorConfidence) * wProp;

              float delta = p->curAv * p->curAv
                          - 4.0f * manager.k2
                            * (manager.k1 * fabsf(p->curAv)
                               + manager.k3 / 4.0f
                               - wt * allocatablePower);
              if (float_equal(delta, 0.0f))
                  newTorqueCurrent[i] = -p->curAv / (2.0f * manager.k2) / k0;
              else if (delta > 0.0f)
                  newTorqueCurrent[i] = (p->pidOutput > 0.0f)
                      ? (-p->curAv + sqrtf(delta)) / (2.0f * manager.k2) / k0
                      : (-p->curAv - sqrtf(delta)) / (2.0f * manager.k2) / k0;
              else
                  newTorqueCurrent[i] = -p->curAv / (2.0f * manager.k2) / k0;
              newTorqueCurrent[i] = _constrain(newTorqueCurrent[i],
                                               -p->pidMaxOutput, p->pidMaxOutput);
          }
      }
      else
      {
          for (int i = 0; i < 4; i++)
              newTorqueCurrent[i] = isMotorConnected(i) ? objs[i]->pidOutput : 0.0f;
      }

      return newTorqueCurrent;
  }

  // ===================== 错误状态检测 =====================
  static void updateErrorFlags(void)
  {
  #if USE_SUPER_CAPACITOR
      if (super_cap.capacitor_voltage > 5.0f)
          clear_flag(&manager.error, FLAG_CAPDisConnect);
      else
          set_flag(&manager.error, FLAG_CAPDisConnect);
  #else
      set_flag(&manager.error, FLAG_CAPDisConnect);
  #endif

      if (referee_data.robot.chassis_power_limit > 0)
          clear_flag(&manager.error, FLAG_RefereeDisConnect);
      else
          set_flag(&manager.error, FLAG_RefereeDisConnect);

      if (!isAllMotorConnected())
          set_flag(&manager.error, FLAG_MotorDisconnect);
      else
          clear_flag(&manager.error, FLAG_MotorDisconnect);
  }

  // ===================== 功率控制更新函数 =====================
  void MasterPower_Update(float boost_value)
  {
      updateErrorFlags();

      boost = boost_value;

      // ----- 超电能量估算 -----
  #if USE_SUPER_CAPACITOR
      if (!is_flagged(&manager.error, FLAG_CAPDisConnect))
          manager.estimatedCapEnergy = (float)super_cap.capacitor_level / 100.0f * 2100.0f;
      else
          manager.estimatedCapEnergy = 0.0f;
  #else
      manager.estimatedCapEnergy = 0.0f;
  #endif

      // ----- 缓冲能量来源 -----
  #if USE_SUPER_CAPACITOR
      if (!is_flagged(&manager.error, FLAG_CAPDisConnect))
      {
          manager.powerBuff   = (float)super_cap.capacitor_level;
          manager.fullBuffSet = CAP_FULL_BUFF_SET;
      }
      else
  #endif
      {
          if (!is_flagged(&manager.error, FLAG_RefereeDisConnect))
              manager.powerBuff = get_referee_buffer_energy();
          manager.fullBuffSet = REFEREE_FULL_BUFF_SET;
      }

      // ----- refereeMaxPower: 裁判在线用裁判数据，离线用 boost -----
      if (!is_flagged(&manager.error, FLAG_RefereeDisConnect))
      {
          float refLimit = get_referee_chassis_power_limit();
          manager.refereeMaxPower = (refLimit > CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD)
                                    ? refLimit : CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD;
      }
      else
      {
          manager.refereeMaxPower = boost;
      }

      // ----- 功率环（能量 PD 控制）-----
      if (is_flagged(&manager.error, FLAG_CAPDisConnect)
          && is_flagged(&manager.error, FLAG_RefereeDisConnect))
      {
          // 双断线：保守固定比例
          manager.fullMaxPower = manager.refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
          PID_Clear(&powerPD_full);
      }
      else
      {
          PID_Calc(&powerPD_full, sqrtf(manager.fullBuffSet), sqrtf(manager.powerBuff));
          manager.fullMaxPower = manager.refereeMaxPower - powerPD_full.Out;
          // 功率环只降不升，上限不超过 refereeMaxPower
          if (manager.fullMaxPower > manager.refereeMaxPower)
              manager.fullMaxPower = manager.refereeMaxPower;
          float minPower = manager.refereeMaxPower * 0.8f;
          if (manager.fullMaxPower < minPower)
              manager.fullMaxPower = minPower;
      }

      // ----- 电机功率估算 -----
      float effectivePower = 0.0f;
      float sumAbsOmega    = 0.0f;
      float sumTorqueSq    = 0.0f;
      const float k0 = MOTOR_KA * MOTOR_MAX_CURRENT / MOTOR_MAX_OUTPUT;

      for (int i = 0; i < 4; i++)
      {
          if (isMotorConnected(i))
              motorDisconnectCounter[i] = 0U;
          else
              motorDisconnectCounter[i]++;

          if (motorDisconnectCounter[i] > 1000U)
              motorDisconnectCounter[i] = 1000U;

          if (motorDisconnectCounter[i] < 1000U && manager.motors[i] != NULL)
          {
              float omega  = rpm2av(manager.motors[i]->vel) / 19.0f;
              float torque = manager.motors[i]->current * k0;
              effectivePower += torque * omega;
              sumAbsOmega    += fabsf(omega);
              sumTorqueSq    += torque * torque;
          }
      }
      manager.estimatedPower = manager.k1 * sumAbsOmega
                             + manager.k2 * sumTorqueSq
                             + effectivePower
                             + manager.k3;

      // 优先用功率计实测数据（RLS 需要），否则用估算
      if (power_receive_data.pm_voltage > 0.1f)
          manager.measuredPower = power_receive_data.pm_power;
      else
          manager.measuredPower = manager.estimatedPower;

      // ----- 更新功率状态 -----
      powerStatus.effectivePower     = effectivePower;
      powerStatus.powerLoss          = manager.measuredPower - effectivePower;
      powerStatus.efficiency         = (manager.measuredPower > 0.1f)
          ? _constrain(effectivePower / manager.measuredPower, 0.0f, 1.0f) : 0.0f;
      powerStatus.estimatedCapEnergy = (uint8_t)(manager.estimatedCapEnergy / 2100.0f * 255.0f);
      powerStatus.error              = manager.error;

      // ----- RLS 在线辨识 k1/k2 -----
      if (manager.rls_enable && fabsf(manager.measuredPower) > 2.0f
          && !(is_flagged(&manager.error, FLAG_CAPDisConnect) && manager.estimatedPower < 0))
      {
          RLS2D_Update(&manager.rls, sumAbsOmega, sumTorqueSq,
                       manager.measuredPower - effectivePower - manager.k3,
                       &manager.k1, &manager.k2);
      }

      // ----- 向超电发送限制功率 -----
  #if USE_SUPER_CAPACITOR
      {
          static uint8_t cap_send_counter = 0;
          if (++cap_send_counter >= 100)
          {
              cap_send_counter = 0;
              super_cap_limitpower_sent(&hfdcan2, 100.0f);
          }
      }
  #endif
    }
  // ===================== 初始化 =====================
  void MasterPower_Init(Motor_feedback *motors[4],float k1, float k2, float k3)
  {
      if (isInitialized) return;

      for (int i = 0; i < 4; i++)
          manager.motors[i] = motors[i];

      manager.k1                = k1;
      manager.k2                = k2;
      manager.k3                = k3;
      manager.error             = 0U;
      manager.refereeMaxPower   = CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD;
      manager.fullMaxPower      = CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD;
      manager.estimatedCapEnergy = 0.0f;

      PID_Init(&powerPD_full, POWER_PD_KP, 0.0f, 0.2f, 0.0f, MAX_CAP_POWER_OUT, 0.0f);

      RLS2D_Init(&manager.rls, 1e-5f, 0.999f);
      RLS2D_SetParams(&manager.rls, k1, k2);
      manager.rls_enable = 0;

      manager.torqueConst    = MOTOR_KA;
      isInitialized          = 1;
  }
#endif
