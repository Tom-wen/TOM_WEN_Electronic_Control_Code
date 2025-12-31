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
  timed_return_motor_status(&hcan1, 4,100);
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
	gimbal->given_current=PID_Calc_Speed(&gimbal->gimbal_motor_relative_angle_pid,gimbal->relative_angle_set,gimbal->gimbal_motor_measure.motor_data.motor.position);
	
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

void motor_control_vel(CAN_HandleTypeDef *hcan, uint8_t id, int16_t vel, int16_t tqe)
{
    uint8_t tdata[8] = {0x07, 0x07, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00};

    *(int16_t *)&tdata[4] = vel;
    *(int16_t *)&tdata[6] = tqe;

    can_send(hcan, 0x8000 | id, tdata, 8);
}

void CAN_cmd_gimbal(gimbal_motor_t* gimbal)
{
motor_control_vel(&hcan1, gimbal->gimbal_motor_measure.motor_data.motor.id,gimbal->given_current,200);
}


void gimbal_set_contorl(void)
{ 
	gimbal_motor_t_yaw.relative_angle_set = 0.7f;
 // gimbal_motor_t_yaw.motor_gyro_set = 500.0f;
}

void timed_return_motor_status(CAN_HandleTypeDef *hcan, uint8_t id, int16_t t_ms)
{
    uint8_t tdata[] = {0x05, 0xb4, 0x02, 0x00, 0x00};

    *(int16_t *)&tdata[3] = t_ms;

    can_send(hcan, 0x8000 | id, tdata, sizeof(tdata));
}
