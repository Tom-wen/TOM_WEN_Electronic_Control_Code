#include "GQ_Motor.h"
#include "remote_control.h"


CAN_TxHeaderTypeDef tx_header;
gimbal_motor_t gimbal_motor_t_yaw;
gimbal_motor_t gimbal_motor_t_pitch;
void gimbal_yaw_init(void)
{
	// 初始化普通PID
	PID_init(&gimbal_motor_t_yaw.gimbal_motor_absolute_angle_pid,yaw_absolute_angle_kp,yaw_absolute_angle_ki,yaw_absolute_angle_kd,yaw_absolute_angle_imax,yaw_absolute_angle_out_max);
	PID_init(&gimbal_motor_t_yaw.gimbal_motor_gyro_pid,yaw_gyro_kp,yaw_gyro_ki,yaw_gyro_kd,yaw_gyro_imax,yaw_gyro_out_max);
	PID_init(&gimbal_motor_t_yaw.gimbal_motor_relative_angle_pid,yaw_relative_angle_kp,yaw_relative_angle_ki,yaw_relative_angle_kd,yaw_relative_angle_imax,yaw_relative_angle_out_max);
	
	// 初始化模糊PID - 位置环
	FuzzyPID_init(&gimbal_motor_t_yaw.gimbal_motor_relative_angle_fuzzy_pid,
	              yaw_fuzzy_relative_angle_kp, yaw_fuzzy_relative_angle_ki, yaw_fuzzy_relative_angle_kd,
	              yaw_fuzzy_relative_angle_out_max,
	              yaw_fuzzy_relative_angle_e_max, yaw_fuzzy_relative_angle_e_min,
	              yaw_fuzzy_relative_angle_ec_max, yaw_fuzzy_relative_angle_ec_min,
	              yaw_fuzzy_relative_angle_kp_max, yaw_fuzzy_relative_angle_kp_min,
	              yaw_fuzzy_relative_angle_ki_max, yaw_fuzzy_relative_angle_ki_min,
	              yaw_fuzzy_relative_angle_kd_max, yaw_fuzzy_relative_angle_kd_min);
	
	// 初始化模糊PID - 速度环
	FuzzyPID_init(&gimbal_motor_t_yaw.gimbal_motor_gyro_fuzzy_pid,
	              yaw_fuzzy_gyro_kp, yaw_fuzzy_gyro_ki, yaw_fuzzy_gyro_kd,
	              yaw_fuzzy_gyro_out_max,
	              yaw_fuzzy_gyro_e_max, yaw_fuzzy_gyro_e_min,
	              yaw_fuzzy_gyro_ec_max, yaw_fuzzy_gyro_ec_min,
	              yaw_fuzzy_gyro_kp_max, yaw_fuzzy_gyro_kp_min,
	              yaw_fuzzy_gyro_ki_max, yaw_fuzzy_gyro_ki_min,
	              yaw_fuzzy_gyro_kd_max, yaw_fuzzy_gyro_kd_min);
	
  gimbal_motor_t_yaw.gimbal_motor_measure.motor_data.motor.id = 0x004;
  gimbal_motor_t_yaw.max_relative_angle = yaw_max_relative_angle;
  gimbal_motor_t_yaw.min_relative_angle = yaw_min_relative_angle;

}

void gimbal_pitch_init(void)
{
	// 初始化普通PID
	PID_init(&gimbal_motor_t_pitch.gimbal_motor_absolute_angle_pid,pitch_absolute_angle_kp,pitch_absolute_angle_ki,pitch_absolute_angle_kd,pitch_absolute_angle_imax,pitch_absolute_angle_out_max);
	PID_init(&gimbal_motor_t_pitch.gimbal_motor_gyro_pid,pitch_gyro_kp,pitch_gyro_ki,pitch_gyro_kd,pitch_gyro_imax,pitch_gyro_out_max);
	PID_init(&gimbal_motor_t_pitch.gimbal_motor_relative_angle_pid,pitch_relative_angle_kp,pitch_relative_angle_ki,pitch_relative_angle_kd,pitch_relative_angle_imax,pitch_relative_angle_out_max);
	
	// 初始化模糊PID - 位置环
	FuzzyPID_init(&gimbal_motor_t_pitch.gimbal_motor_relative_angle_fuzzy_pid,
	              pitch_fuzzy_relative_angle_kp, pitch_fuzzy_relative_angle_ki, pitch_fuzzy_relative_angle_kd,
	              pitch_fuzzy_relative_angle_out_max,
	              pitch_fuzzy_relative_angle_e_max, pitch_fuzzy_relative_angle_e_min,
	              pitch_fuzzy_relative_angle_ec_max, pitch_fuzzy_relative_angle_ec_min,
	              pitch_fuzzy_relative_angle_kp_max, pitch_fuzzy_relative_angle_kp_min,
	              pitch_fuzzy_relative_angle_ki_max, pitch_fuzzy_relative_angle_ki_min,
	              pitch_fuzzy_relative_angle_kd_max, pitch_fuzzy_relative_angle_kd_min);
	
	// 初始化模糊PID - 速度环
	FuzzyPID_init(&gimbal_motor_t_pitch.gimbal_motor_gyro_fuzzy_pid,
	              pitch_fuzzy_gyro_kp, pitch_fuzzy_gyro_ki, pitch_fuzzy_gyro_kd,
	              pitch_fuzzy_gyro_out_max,
	              pitch_fuzzy_gyro_e_max, pitch_fuzzy_gyro_e_min,
	              pitch_fuzzy_gyro_ec_max, pitch_fuzzy_gyro_ec_min,
	              pitch_fuzzy_gyro_kp_max, pitch_fuzzy_gyro_kp_min,
	              pitch_fuzzy_gyro_ki_max, pitch_fuzzy_gyro_ki_min,
	              pitch_fuzzy_gyro_kd_max, pitch_fuzzy_gyro_kd_min);
	
  gimbal_motor_t_pitch.max_relative_angle = pitch_max_relative_angle;
  gimbal_motor_t_pitch.min_relative_angle = pitch_min_relative_angle;

}


void update_pitch(gimbal_motor_t* gimbal,motor_measure_t* M6020_motor_measure_t,CAN_RxHeaderTypeDef* rx_header)
{
   gimbal->gimbal_motor_measure.motor_data.motor.id = rx_header->StdId >> 8;
   gimbal->relative_angle = motor_ecd_to_angle_change(M6020_motor_measure_t->ecd,0)/360.0f;//转化为圈数
   gimbal->motor_gyro=M6020_motor_measure_t->speed_rpm;
}

void update_yaw(gimbal_motor_t* gimbal,uint8_t rx_data[24],CAN_RxHeaderTypeDef* rx_header)
{
	 uint8_t len = 0;	
	 len = rx_header->DLC;
	 memcpy(&gimbal->gimbal_motor_measure.motor_data.data[4], &rx_data[2], len - 2);  
   gimbal->gimbal_motor_measure.motor_data.motor.position = (*(int16_t *)&rx_data[2]) * 0.0001f;
   gimbal->gimbal_motor_measure.motor_data.motor.velocity = (*(int16_t *)&rx_data[4]) * 0.00025f;
   gimbal->gimbal_motor_measure.motor_data.motor.torque = (*(int16_t *)&rx_data[6]) * 0.004563f;
   gimbal->relative_angle=gimbal->gimbal_motor_measure.motor_data.motor.position;
   gimbal->motor_gyro=gimbal->gimbal_motor_measure.motor_data.motor.velocity;

}

void gimbal_pitch_PID_Calc(gimbal_motor_t* gimbal)
{
	gimbal->motor_gyro_set=PID_Calc_Angle(&gimbal->gimbal_motor_relative_angle_pid,gimbal->relative_angle_set,gimbal->relative_angle);
	gimbal->given_current=PID_Calc_Speed(&gimbal->gimbal_motor_gyro_pid,gimbal->motor_gyro_set,gimbal->motor_gyro);
}	

void gimbal_yaw_PID_Calc(gimbal_motor_t* gimbal)
{

	gimbal->motor_gyro_set=PID_Calc_Angle(&gimbal->gimbal_motor_relative_angle_pid,gimbal->relative_angle_set,gimbal->relative_angle);
 	gimbal->given_current=PID_Calc_Speed(&gimbal->gimbal_motor_gyro_pid,gimbal->motor_gyro_set,gimbal->motor_gyro);


}



uint8_t can_send(CAN_HandleTypeDef* hcan,uint16_t id,uint8_t *msg,uint16_t len)
{
  if(id > 0x7FF)
  {
    tx_header.IDE=CAN_ID_EXT;
    tx_header.ExtId=id;
  }
  else
  {
    tx_header.IDE=CAN_ID_STD;
    tx_header.StdId=id;
	}
	tx_header.RTR=CAN_RTR_DATA;
	tx_header.DLC=len;

	if(HAL_CAN_AddTxMessage(hcan, &tx_header, msg, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	{
    if(HAL_CAN_AddTxMessage(hcan, &tx_header, msg, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
    {
      HAL_CAN_AddTxMessage(hcan, &tx_header, msg, (uint32_t*)CAN_TX_MAILBOX2);
    }
  }

  return 0;

}


/**
 * @brief 将当前位置设为零位（注意此操作只在RAM中修改，需要执行 `conf write` 指令保存到flash中）
 * @param id 电机ID
 */
void rezero_pos(CAN_HandleTypeDef *hcan, uint8_t id)
{
    uint8_t tdata[] = {0x40, 0x01, 0x04, 0x64, 0x20, 0x63, 0x0a};

    can_send(hcan, 0x8000 | id, tdata, sizeof(tdata));
     // 等待超时1s

 // 发送数据
}
/**
 * @brief 将 RAM 中参数写入到 flash 中(使用此指令后需要重新启动系统)
 * @param id 电机ID
 */
void conf_write(CAN_HandleTypeDef *hcan, uint8_t id)
{
    uint8_t tdata[] = {0x05, 0xb3, 0x02, 0x00, 0x00};

    can_send(hcan, 0x8000 | id, tdata, sizeof(tdata));
}


/**
 * @brief DQ电流控制
 * @param id 电机ID
 * @param cur Q相电流，单位：0.1A，如 cur = 10 表示 Q 相电压为 1A
 */
void motor_control_cur(CAN_HandleTypeDef *hcan, uint8_t id, int16_t cur)
{
    static uint8_t tdata[] = {0x01, 0x00, 0x09, 0x05, 0x1c, 0x00, 0x00};

    // *(int16_t *)&tdata[5] = cur;
    memcpy(&tdata[5], &cur, sizeof(int16_t));

    can_send(hcan, id, tdata, sizeof(tdata));
}

void CAN_yaw_cmd_gimbal(gimbal_motor_t* gimbal)
{
motor_control_cur(&hcan1, gimbal->gimbal_motor_measure.motor_data.motor.id,gimbal->given_current);
}

void CAN_pitch_cmd_gimbal(gimbal_motor_t* gimbal)
{
M6020_send((int16_t)gimbal->given_current);
}


void gimbal_set_control_init(void)
{ 
	gimbal_motor_t_yaw.relative_angle_set = 0.0f;//设定初始位置0.5圈
  gimbal_motor_t_pitch.relative_angle_set = 0.0f;

}
void gimbal_set_control(void)
{ 
  //	gimbal_motor_t_yaw.relative_angle_set = 0.52f;//设定初始位置0.5圈
  // // 云台偏航相对角度设置
  // if (gimbal_motor_t_yaw.relative_angle_set > gimbal_motor_t_yaw.max_relative_angle)
  // {
  //   gimbal_motor_t_yaw.relative_angle_set = gimbal_motor_t_yaw.max_relative_angle;
  // }
  // else if (gimbal_motor_t_yaw.relative_angle_set < gimbal_motor_t_yaw.min_relative_angle)
  // {
  //   gimbal_motor_t_yaw.relative_angle_set = gimbal_motor_t_yaw.min_relative_angle;
  // }
// //// 发送数据给雷达
//   gimbal_motor_t_yaw.relative_angle_set += -gimbal_motor_t_yaw.radar_add;
//   gimbal_motor_t_pitch.relative_angle_set += gimbal_motor_t_pitch.radar_add;
// 遥控控制
   RemoteControl(&gimbal_motor_t_yaw,&gimbal_motor_t_pitch);


}

void timed_return_motor_status(CAN_HandleTypeDef *hcan, uint8_t id, int16_t t_ms)
{
    uint8_t tdata[] = {0x05, 0xb4, 0x02, 0x00, 0x00};

    *(int16_t *)&tdata[3] = t_ms;

    can_send(hcan, 0x8000 | id, tdata, sizeof(tdata));
}

/**
 * @brief 读取电机位置、速度、扭矩等信息
 * @param id 电机ID
 */
void motor_read(CAN_HandleTypeDef *hcan, uint8_t id)
{
    static uint8_t tdata[8] = {0x17, 0x01};

    can_send(hcan, 0x8000 | id, tdata, sizeof(tdata));
}



//M6020电机参数配置函数
static uint8_t M6020_can_send_data[8];
static CAN_TxHeaderTypeDef  tx_message;
void M6020_send(uint16_t data)
{
	uint32_t send_mail_box;
    tx_message.StdId = 0x2FF;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    M6020_can_send_data[0] = 0;
	M6020_can_send_data[1] = 0;
	M6020_can_send_data[2] = data >> 8;
	M6020_can_send_data[3] = data;
	M6020_can_send_data[4] = 0;
	M6020_can_send_data[5] = 0;
	M6020_can_send_data[6] = 0;
	M6020_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, M6020_can_send_data, &send_mail_box);
}


/**
 * @brief          根据ecd和offset_ecd计算角度变化
 * @param[in]      ecd: 电机编码器数值
 * @param[in]      offset_ecd: 电机中位编码器数值
 * @retval         编码器角度，单位rad
 */
static float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
  int32_t relative_ecd = ecd - offset_ecd;
  
  // 处理编码器过零
  if (relative_ecd > 4096)
  {
    relative_ecd -= 8191;
  }
  else if (relative_ecd < -4096)
  {
    relative_ecd += 8191; 
  }

  return relative_ecd / 22.75278;
}


motor_measure_t M6020_motor_measure;
