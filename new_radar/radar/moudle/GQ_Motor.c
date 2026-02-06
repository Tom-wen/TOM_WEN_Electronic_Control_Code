#include "GQ_Motor.h"
#include "remote_control.h"
#include <stdbool.h>  


gimbal_motor_t gimbal_motor_t_yaw;
gimbal_motor_t gimbal_motor_t_pitch;




/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF,CAN_HandleTypeDef* hcan)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 13,hcan);
}


/**
  * @brief    修改开环/闭环控制模式（Y42）
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式，默认为1,0为开环模式，1为闭环FOC模式
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode,CAN_HandleTypeDef* hcan)
{
  __IO static uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x46;                       // 功能码
  cmd[2] =  0x69;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  ctrl_mode;                  // 控制模式，默认为1,0为开环模式，1为闭环FOC模式
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 6,hcan);
}

/**
  * @brief    修改PID参数
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    kp 	 		 ：比例系数，默认为Y42/18000
	* @param    ki 	 		 ：积分系数，默认为Y42/10
	* @param    kd 	 		 ：微分系数，默认为Y42/18000
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki, uint32_t kd,CAN_HandleTypeDef* hcan)
{
  __IO static uint8_t cmd[20] = {0};
  
  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0x4A;                      // 功能码
  cmd[2]  =  0xC3;                      // 辅助码
  cmd[3]  =  svF;                       // 是否存储标志，false为不存储，true为存储
  cmd[4]  =  (uint8_t)(kp >> 24);				// kp
	cmd[5]  =  (uint8_t)(kp >> 16);
	cmd[6]  =  (uint8_t)(kp >> 8);
	cmd[7]  =  (uint8_t)(kp >> 0);
	cmd[8]  =  (uint8_t)(ki >> 24);				// ki
	cmd[9]  =  (uint8_t)(ki >> 16);
	cmd[10] =  (uint8_t)(ki >> 8);
	cmd[11] =  (uint8_t)(ki >> 0);
	cmd[12] =  (uint8_t)(kd >> 24);				// kd
	cmd[13] =  (uint8_t)(kd >> 16);
	cmd[14] =  (uint8_t)(kd >> 8);
	cmd[15] =  (uint8_t)(kd >> 0);
  cmd[16] =  0x6B;                      // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 17,hcan);
}


/**
  * @brief    恢复出厂设置
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Restore_Motor(uint8_t addr,CAN_HandleTypeDef* hcan)
{
  __IO static uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0F;                       // 功能码
  cmd[2] =  0x5F;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
	can_SendCmd(cmd, 4,hcan);
}



