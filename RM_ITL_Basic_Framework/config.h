// config.h （与 CMakeLists.txt 同级）
#ifndef __CONFIG_H__
#define __CONFIG_H__



// #define AUTO_SENTRY             //哨兵
// ==================== 板子类型选择 ====================
// #define chassis_board    // 底盘控制板
// #define gimbal_board        // 云台控制板

// ==================== 底盘类型 ====================
//#define COMPILE_AGV_CHASSIS      // 舵轮底盘
//  #define COMPILE_MECANUM_CHASSIS     // 麦克纳姆轮底盘
#define ROBOTIC_SWING_CHASSIS //摆臂底盘
 #define ROBOTIC_SWING_HOISTING //摆臂
//#define COMPILE_HERO_CHASSIS     //英雄底盘
//#define COMPILE_M_HERO_CHASSIS     //麦轮英雄底盘
// #define TARGET_CHASSIS      //靶车


// ==================== 云台类型 ====================
//  #define COMPILE_GIMBAL              // 步兵云台
#define ROBOTIC_SWING_GIMBAL        // 摆臂云台
//#define COMPILE_UAV_GIMBAL       // 无人机云台
//#define COMPILE_HERO_GIMBAL        // 英雄云台

// ==================== 发射机构 ====================
 //#define COMPILE_SHOOT               // 步兵发射
#define ROBOTIC_SWING_SHOOT        // 摆臂发射
// #define COMPILE_UAV_SHOOT        // 无人机发射
//#define COMPILE_HERO_SHOOT       // 英雄发射
//  #define ROBOTIC_SWING_SHOOT       // 摆臂发射

// ==================== 机械臂 ====================
// #define COMPILE_ARM              // 机械臂
// #define COMPILE_ARM_LIFT         // 机械臂抬升

// ==================== 其他功能 ====================
// #define COMPILE_TEST             // 测试任务
// #define COMPILE_REFEREE             // 裁判系统
 //#define COMPILE_UI
 //#define COMPILE_POWER               // 功率控制
//  #define COMPILE_M_POWER               // 功率控制
 //#define HOT_CONTROL                 // 热量控制

// ==================== 遥控器类型 ====================
#define DJI_REMOTE               // 大疆遥控器
//#define FS_REMOTE                // 富斯遥控器
//#define VT_03_REMOTE                // 图传遥控器
// =================== PID类型 =======================
#define PRE_PID               //之前PID
// #define ALG_PID                 //中科大PID

#endif
