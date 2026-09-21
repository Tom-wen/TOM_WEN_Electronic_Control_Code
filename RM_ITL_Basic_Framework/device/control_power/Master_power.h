#ifndef __MASTER_POWER_H__
#define __MASTER_POWER_H__

  #include "main.h"
  #include "FreeRTOS.h"
  #include "task.h"
  #include "Pid.h"
  #include "control_power.h"
  #include "motor_types.h"
  #include "rls.h"
#ifdef COMPILE_M_POWER
  // ===================== 超电开关：0=不用超电，1=用超电 =====================
  #define USE_SUPER_CAPACITOR  0   

  // ===================== 功率控制参数 =====================
  #define POWER_PD_KP                               50.0f
  #define MAX_CAP_POWER_OUT                         230.0f
  #define CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD 10.0f
  #define CAP_REFEREE_BOTH_GG_COE                   0.90f

  // 能量环参数
  #define REFEREE_FULL_BUFF_SET  60.0f
  #define CAP_FULL_BUFF_SET      90.0f   // 超电百分比 0~100，满电 90% 以上不再限功率

  // 功率分配阈值
  #define ERROR_POWER_DISTRIBUTION_SET  20.0f
  #define PROP_POWER_DISTRIBUTION_SET   15.0f

  // 电机参数（M3508，电机轴坐标系）
  // vel 单位: RPM (motor shaft)，current 单位: raw ±16384 = ±20A
  #define MOTOR_KA          0.3f     // 转矩常数 N·m/A (motor shaft)
  #define MOTOR_MAX_CURRENT 20.0f    // 最大电流 A
  #define MOTOR_MAX_OUTPUT  16384.0f // 最大输出原始值
  #define RPM_TO_RADS       0.10472f // RPM → rad/s 系数

  // ===================== 类型定义 =====================

  typedef enum {
      FLAG_MotorDisconnect   = 1U,
      FLAG_RefereeDisConnect = 2U,
      FLAG_CAPDisConnect     = 4U
  } ErrorFlags;

  typedef struct {
      uint8_t     error;
      Motor_feedback *motors[4]; // 四个底盘电机反馈指针

      float powerBuff;       // 当前缓冲能量
      float fullBuffSet;     // 能量环目标值
      float fullMaxPower;    // 功率环计算出的功率上限 (W)
      float refereeMaxPower; // 裁判系统/boost 基础功率上限

      float measuredPower;   // 实测功率
      float estimatedPower;  // 估算功率
      float estimatedCapEnergy;

      float torqueConst; // = MOTOR_KA（使用电机轴坐标）
      float k1;          // 速度摩擦损耗系数
      float k2;          // 电流铜损系数
      float k3;          // 固定损耗 (W)
      uint8_t rls_enable;  // 0=关闭, 1=开启 RLS 在线辨识
      RLS2D rls;
  } Manager;

  typedef struct {
      float pidOutput;    // PID输出（原始电流指令，±16384）
      float curAv;        // 当前电机角速度 rad/s (motor shaft)
      float setAv;        // 目标电机角速度 rad/s (motor shaft)
      float pidMaxOutput; // PID最大输出
  } PowerObj;

  typedef struct {
      float   maxPowerLimited;
      float   sumPowerCmd_before_clamp;
      float   effectivePower;
      float   powerLoss;
      float   efficiency;
      uint8_t estimatedCapEnergy;
      uint8_t error;
  } PowerStatus;

  //外部声明
  extern pm_power power_receive_data;
  extern super_cap_data super_cap;

  // ===================== 接口 =====================
  // motors: 4个 Motor_feedback* 指针（底盘四轮顺序：RF, LF, LB, RB）
  // k1/k2/k3: 功率模型参数（不确定时用默认值 0.22, 1.2, 2.78）
  void MasterPower_Init(Motor_feedback *motors[4],
                        float k1, float k2, float k3);
  void     MasterPower_Update(float boost_value);
  void     MasterPower_SetRLSEnable(uint8_t enable);
  float   *getControlledOutput(PowerObj *objs[4]);
  const volatile PowerStatus *getPowerStatus(void);
  float    getLatestFeedbackJudgePowerLimit(void);

  float get_referee_chassis_power_limit(void);
  float get_referee_buffer_energy(void);

 #endif

  #endif /* __MASTER_POWER_H__ */
