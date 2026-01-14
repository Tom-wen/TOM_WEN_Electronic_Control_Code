#include "GQ_Motor.h"


CAN_TxHeaderTypeDef tx_header;
gimbal_motor_t gimbal_motor_t_yaw;
gimbal_motor_t gimbal_motor_t_pitch;
void gimbal_yaw_init(void)
{
	PID_init(&gimbal_motor_t_yaw.gimbal_motor_absolute_angle_pid,yaw_absolute_angle_kp,yaw_absolute_angle_ki,yaw_absolute_angle_kd,yaw_absolute_angle_imax,yaw_absolute_angle_out_max);
	PID_init(&gimbal_motor_t_yaw.gimbal_motor_gyro_pid,yaw_gyro_kp,yaw_gyro_ki,yaw_gyro_kd,yaw_gyro_imax,yaw_gyro_out_max);
	PID_init(&gimbal_motor_t_yaw.gimbal_motor_relative_angle_pid,yaw_relative_angle_kp,yaw_relative_angle_ki,yaw_relative_angle_kd,yaw_relative_angle_imax,yaw_relative_angle_out_max);
  gimbal_motor_t_yaw.gimbal_motor_measure.motor_data.motor.id = 0x004;
  gimbal_motor_t_yaw.max_relative_angle = yaw_max_relative_angle;
  gimbal_motor_t_yaw.min_relative_angle = yaw_min_relative_angle;
  timed_return_motor_status(&hcan1, 4, 10);
}

void gimbal_pitch_init(void)
{
	PID_init(&gimbal_motor_t_pitch.gimbal_motor_absolute_angle_pid,pitch_absolute_angle_kp,pitch_absolute_angle_ki,pitch_absolute_angle_kd,pitch_absolute_angle_imax,pitch_absolute_angle_out_max);
	PID_init(&gimbal_motor_t_pitch.gimbal_motor_gyro_pid,pitch_gyro_kp,pitch_gyro_ki,pitch_gyro_kd,pitch_gyro_imax,pitch_gyro_out_max);
	PID_init(&gimbal_motor_t_pitch.gimbal_motor_relative_angle_pid,pitch_relative_angle_kp,pitch_relative_angle_ki,pitch_relative_angle_kd,pitch_relative_angle_imax,pitch_relative_angle_out_max);
  gimbal_motor_t_pitch.gimbal_motor_measure.motor_data.motor.id = 0x001;
  gimbal_motor_t_pitch.max_relative_angle = pitch_max_relative_angle;
  gimbal_motor_t_pitch.min_relative_angle = pitch_min_relative_angle;
  timed_return_motor_status(&hcan1, 1, 10);
}


void update(gimbal_motor_t* gimbal,uint8_t rx_data[24],CAN_RxHeaderTypeDef* rx_header)
{
	 uint8_t len = 0;	
	 len = rx_header->DLC;
	
	
   gimbal->gimbal_motor_measure.motor_data.motor.id = rx_header->StdId >> 8;
	 memcpy(&gimbal->gimbal_motor_measure.motor_data.data[4], &rx_data[2], len - 2);  
   gimbal->gimbal_motor_measure.motor_data.motor.position = (*(int16_t *)&rx_data[2]) * 0.0001f;
   gimbal->gimbal_motor_measure.motor_data.motor.velocity = (*(int16_t *)&rx_data[4]) * 0.00025f;
   gimbal->gimbal_motor_measure.motor_data.motor.torque = (*(int16_t *)&rx_data[6]) * 0.004563f;
}

void gimbal_PID_Calc(gimbal_motor_t* gimbal)
{
	
	//gimbal->given_current=PID_Calc_Speed(&gimbal->gimbal_motor_gyro_pid,gimbal->motor_gyro_set,gimbal->gimbal_motor_measure.motor_data.motor.velocity/0.00025f);
	gimbal->motor_gyro_set=PID_Calc_Angle(&gimbal->gimbal_motor_relative_angle_pid,gimbal->relative_angle_set,gimbal->gimbal_motor_measure.motor_data.motor.position);//位锟矫伙拷
  gimbal->given_current=PID_Calc_Speed(&gimbal->gimbal_motor_gyro_pid,gimbal->motor_gyro_set,gimbal->gimbal_motor_measure.motor_data.motor.velocity);//锟劫度伙拷

	
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
 * @brief 锟劫度匡拷锟斤拷
 * @param id 锟斤拷锟絀D
 * @param vel 锟劫度ｏ拷锟斤拷位 0.00025 转/锟诫，锟斤拷 val = 1000 锟斤拷示 0.25 转/锟斤拷
 * @param tqe 锟斤拷锟截ｏ拷锟斤拷位锟斤拷0.01 NM锟斤拷锟斤拷 torque = 110 锟斤拷示锟斤拷锟斤拷锟斤拷锟轿? 1.1NM
 * @note 锟劫度猴拷锟斤拷锟截诧拷锟斤拷同时为0
 */
void motor_control_vel(CAN_HandleTypeDef *hcan, uint8_t id, int16_t vel, int16_t tqe)
{
    uint8_t tdata[8] = {0x07, 0x07, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00};

    *(int16_t *)&tdata[4] = vel;
    *(int16_t *)&tdata[6] = tqe;

    can_send(hcan, 0x8000 | id, tdata, 8);
}

/**
 * @brief 速度控制
 * @param id 电机ID
 * @param vel 速度，单位 0.00025 转/分，例如 val = 1000 表示 0.25 转/分
 * @param tqe 扭矩，单位为0.01 NM，例如 torque = 110 表示扭矩为 1.1NM
 * @note 速度和扭矩不能同时为0
 */
void motor_control_pos(CAN_HandleTypeDef *hcan, uint8_t id, int32_t pos, int16_t tqe)
{
    uint8_t tdata[8] = {0x07, 0x07, 0x0A, 0x05, 0x00, 0x00, 0x80, 0x00};

    *(int16_t *)&tdata[2] = pos;
    *(int16_t *)&tdata[6] = tqe;

    can_send(hcan, 0x8000 | id, tdata, 8);
}

void CAN_cmd_gimbal(gimbal_motor_t* gimbal)
{
motor_control_vel(&hcan1, gimbal->gimbal_motor_measure.motor_data.motor.id,gimbal->given_current,200);
}


void gimbal_set_control_init(void)
{ 
	gimbal_motor_t_yaw.relative_angle_set = 0.0f;//设定初始位置0.5圈
  gimbal_motor_t_pitch.relative_angle_set = 0.0f;
 // gimbal_motor_t_yaw.motor_gyro_set = 500.0f;
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
// 发送数据给雷达
  gimbal_motor_t_yaw.relative_angle_set += -gimbal_motor_t_yaw.radar_add;
  gimbal_motor_t_pitch.relative_angle_set += gimbal_motor_t_pitch.radar_add;
// 遥控控制
  // RemoteControl(&gimbal_motor_t_yaw,&gimbal_motor_t_pitch);


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
