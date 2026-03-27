
#include "CAN_receive.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "GQ_Motor.h"

#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

float _207_angle[2];

#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }
 motor_measure_t motor_chassis[4];  //底盘电机
static motor_measure_t motor_gimbal[2];   //云台电机
static motor_measure_t motor_trigger;     //拨弹电机
static motor_measure_t motor_friction[4]; //摩擦轮电机
static FDCAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static FDCAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
static FDCAN_TxHeaderTypeDef cap_tx_message;
static uint8_t cap_can_send_data[8];
static void parse_supercap_data(uint8_t *rx_data);
limit_switch_t limit_switch;
// 全局超级电容数据
supercap_data_t supercap_data = {0};

float PowerData[7];
	
int16_t round_207;
float this_angle_207;

/**
 * @brief           计算3508电机累计旋转角度
 * @param[out]      motor:电机结构数据指针
 */
void Calc_motor_Angle(motor_measure_t *motor)
{
  if (motor->ecd - motor->last_ecd > 4095.5)
  {
    motor->round--;
  }
  else if (motor->ecd - motor->last_ecd < -4095.5)
  {
    motor->round++;
  }
  motor->angle = (motor->round * ANGLE_T + motor->last_ecd) * 0.04395067757f/19; //转换为360度一圈
}

/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx_header;

  uint8_t rx_data[24]={0};

  HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rx_header, rx_data);

  if (hcan == &hfdcan2)
  {
    switch (rx_header.Identifier)
    {
    case CAN_3508_M1_ID:
    {
      get_motor_measure(&motor_chassis[0], rx_data); //读取数据
      detect_hook(CHASSIS_MOTOR1_TOE); //错误检测
      break;
    }
    case CAN_3508_M2_ID:
    {
      get_motor_measure(&motor_chassis[1], rx_data);
      detect_hook(CHASSIS_MOTOR2_TOE);
      break;
    }
    case CAN_3508_M3_ID:
    {
      get_motor_measure(&motor_chassis[2], rx_data);
      detect_hook(CHASSIS_MOTOR3_TOE);
      break;
    }
    case CAN_3508_M4_ID:
    {
      get_motor_measure(&motor_chassis[3], rx_data);
      detect_hook(CHASSIS_MOTOR4_TOE);
      break;
    }
    case CAN_YAW_MOTOR_ID:
    {
      get_motor_measure(&motor_gimbal[0], rx_data);
      detect_hook(YAW_GIMBAL_MOTOR_TOE);
      break;
    }



    default:
    {
      break;
    }
    }
  }
  else if (hcan == &hfdcan1)
  {
    switch (rx_header.Identifier)
    {
    case CAN_FRONT_LEFT_FRICTION_ID:
    {
      get_motor_measure(&motor_friction[0], rx_data); //读取数据
      detect_hook(FRONT_LEFT_FRICTION_MOTOR_TOE);           //错误检测
      break;
    }
    case CAN_FRONT_RIGHT_FRICTION_ID:
    {
      get_motor_measure(&motor_friction[1], rx_data); //读取数据
      detect_hook(FRONT_RIGHT_FRICTION_MOTOR_TOE);          //错误检测
      break;
    }
	 case CAN_PIT_MOTOR_ID:
    {
      get_motor_measure(&motor_gimbal[1], rx_data);
      detect_hook(PITCH_GIMBAL_MOTOR_TOE);
      break;
    }
		case CAN_TRIGGER_MOTOR_ID:
    {
      get_motor_measure(&motor_trigger, rx_data);
      Calc_motor_Angle(&motor_trigger);
      detect_hook(TRIGGER_MOTOR_TOE);
      _207_angle[1] = _207_angle[0];
      _207_angle[0] = motor_trigger.ecd * 0.044;
      if (_207_angle[0] - _207_angle[1] > 180)
      {
        round_207--;
      }
      else if (_207_angle[0] - _207_angle[1] < -180)
      {
        round_207++;
      }
      this_angle_207 = round_207 * 360 + _207_angle[0];
      break;
    }


    default:
    {
      break;
    }
    }
  }
  else if (hcan == &hfdcan3)
  {
      if (rx_header.DataLength != 0)
      {
        switch (rx_header.Identifier)
        {
          case CAN_GQ_M1_ID:
          {
            get_GQ_motor_measure(&GQ_Motor_Measure[0], rx_data, 1);
            detect_hook(HOISTING_MOTOR1_TOE);
            break;
          }
          case CAN_GQ_M2_ID:
          {
            get_GQ_motor_measure(&GQ_Motor_Measure[1], rx_data, 2);
            detect_hook(HOISTING_MOTOR2_TOE);
            break;
          }
          case CAN_GQ_M3_ID:
          {
            get_GQ_motor_measure(&GQ_Motor_Measure[2], rx_data, 3);
            detect_hook(HOISTING_MOTOR3_TOE);
            break;
          }
          case CAN_GQ_M4_ID:
          {
            get_GQ_motor_measure(&GQ_Motor_Measure[3], rx_data, 4);
            detect_hook(HOISTING_MOTOR4_TOE);
            break;
          }
          case CAN_SUPERCAP_ID:
          {
            parse_supercap_data(rx_data);
            break;
          }

          case CAN_LIMIMT_ID:
          {
            limit_data_process(&limit_switch,rx_data);
          } 
        }
      }
  }

}

/**
 * @brief 摩擦轮控制量发送函数 can1
 *
 * @param left_friction
 * @param rigit_friction
 * @param rev
 */
void CAN_cmd_friction(int16_t left_friction, int16_t rigit_friction)
{
  gimbal_tx_message.Identifier = CAN_FRICTION_ALL_ID;
  gimbal_tx_message.IdType = FDCAN_STANDARD_ID;
  gimbal_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  gimbal_tx_message.DataLength = 0x08;
  gimbal_can_send_data[0] = 0;
  gimbal_can_send_data[1] = 0;
  gimbal_can_send_data[2] = (left_friction >> 8);
  gimbal_can_send_data[3] = left_friction;
  gimbal_can_send_data[4] = (rigit_friction >> 8);
  gimbal_can_send_data[5] = rigit_friction;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  gimbal_tx_message.ErrorStateIndicator=FDCAN_ESI_PASSIVE;
  gimbal_tx_message.BitRateSwitch=FDCAN_BRS_OFF;
  gimbal_tx_message.FDFormat=FDCAN_CLASSIC_CAN;
  gimbal_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  gimbal_tx_message.MessageMarker=0;
	HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&gimbal_tx_message, gimbal_can_send_data);
}

/**
 * @brief 拨弹电机控制量发送函数   can2    ID 
 *
 * @param shoot
 */
void CAN_cmd_shoot(int16_t shoot)
{
  gimbal_tx_message.Identifier = 0x1FF;
  gimbal_tx_message.IdType = FDCAN_STANDARD_ID;
  gimbal_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  gimbal_tx_message.DataLength = 0x08;
  gimbal_can_send_data[0] = (shoot >> 8);
  gimbal_can_send_data[1] = (shoot >> 8);
  gimbal_can_send_data[2] = 0;
  gimbal_can_send_data[3] = 0;
  gimbal_can_send_data[4] = 0;
  gimbal_can_send_data[5] = 0;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  gimbal_tx_message.ErrorStateIndicator=FDCAN_ESI_PASSIVE;
  gimbal_tx_message.BitRateSwitch=FDCAN_BRS_OFF;
  gimbal_tx_message.FDFormat=FDCAN_CLASSIC_CAN;
  gimbal_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  gimbal_tx_message.MessageMarker=0;
	HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&gimbal_tx_message, gimbal_can_send_data);
}

/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208) can1
 * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
 * @retval         none
 */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch)
{
	// Pitch  Can1    电机ID   6 
  gimbal_tx_message.Identifier = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IdType = FDCAN_STANDARD_ID;
  gimbal_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  gimbal_tx_message.DataLength = 0x08;
	gimbal_can_send_data[0] = 0;
  gimbal_can_send_data[1] = 0;
  gimbal_can_send_data[2] = (pitch >> 8);
  gimbal_can_send_data[3] = pitch;
  gimbal_can_send_data[4] = 0;
  gimbal_can_send_data[5] = 0;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  gimbal_tx_message.ErrorStateIndicator=FDCAN_ESI_PASSIVE;
  gimbal_tx_message.BitRateSwitch=FDCAN_BRS_OFF;
  gimbal_tx_message.FDFormat=FDCAN_CLASSIC_CAN;
  gimbal_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  gimbal_tx_message.MessageMarker=0;
  HAL_StatusTypeDef status_1 = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&gimbal_tx_message, gimbal_can_send_data);

	// Yaw  can2   电机ID   5        
  gimbal_tx_message.Identifier = CAN_GIMBAL_ALL_ID;   
  gimbal_tx_message.IdType = FDCAN_STANDARD_ID;
  gimbal_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  gimbal_tx_message.DataLength = 0x08;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = 0;
  gimbal_can_send_data[3] = 0;
  gimbal_can_send_data[4] = 0;
  gimbal_can_send_data[5] = 0;
  gimbal_can_send_data[6] = 0;
  gimbal_can_send_data[7] = 0;
  gimbal_tx_message.ErrorStateIndicator=FDCAN_ESI_PASSIVE;
  gimbal_tx_message.BitRateSwitch=FDCAN_BRS_OFF;
  gimbal_tx_message.FDFormat=FDCAN_CLASSIC_CAN;
  gimbal_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  gimbal_tx_message.MessageMarker=0;

  HAL_StatusTypeDef status_2 = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&gimbal_tx_message, gimbal_can_send_data);
}

/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204) can1
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  chassis_tx_message.Identifier = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IdType = FDCAN_STANDARD_ID;
  chassis_tx_message.TxFrameType = FDCAN_DATA_FRAME;
  chassis_tx_message.DataLength = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;
	chassis_tx_message.ErrorStateIndicator=FDCAN_ESI_PASSIVE;
  chassis_tx_message.BitRateSwitch=FDCAN_BRS_OFF;
  chassis_tx_message.FDFormat=FDCAN_CLASSIC_CAN;
  chassis_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
  chassis_tx_message.MessageMarker=0;
	
	HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&chassis_tx_message, chassis_can_send_data);
}

/**
 * @brief     发送超级电容设定功率
 * @param Power   30W~250W
 * 10Hz
 */
void super_cap_send_power(uint16_t power)
{
   cap_tx_message.Identifier = CAN_SUPERCAP_ID;
   cap_tx_message.IdType = FDCAN_STANDARD_ID;
   cap_tx_message.TxFrameType = FDCAN_DATA_FRAME;
   cap_tx_message.DataLength = 0x08;
   cap_can_send_data[0]=(uint8_t)power; //整数部分
   cap_can_send_data[1]=(uint8_t)(power*100.0f)%100; //小数部分
   cap_can_send_data[2]=0;
   cap_can_send_data[3]=0;
   cap_can_send_data[4]=0;
   cap_can_send_data[5]=0;
   cap_can_send_data[6]=0;
   cap_can_send_data[7]=0;
   HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3,&cap_tx_message, cap_can_send_data);

}

/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_gimbal[0];
}

/**
 * @brief          返回pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_gimbal[1];
}

/**
 * @brief          返回拨弹电机 2006电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_trigger;
}

const motor_measure_t *get_left_friction_motor_measure_point(void)
{
  return &motor_friction[0];
}

const motor_measure_t *get_right_friction_motor_measure_point(void)
{
  return &motor_friction[1];
}

/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}

void limit_data_process(limit_switch_t *limit_data ,uint8_t *data)
{
      // 检查帧头和帧尾
    if (data[0] == 'F' && data[1] == 'E' && data[6] == 'F')
    {
        limit_data->hoisting_1 = data[2];
        limit_data->hoisting_2 = data[3];
        limit_data->hoisting_3 = data[4];
        limit_data->hoisting_3 = data[5];
    }

}




/**
 * @brief          解析超级电容 CAN 数据
 * @param[in]      rx_data: CAN 接收数据 (8 字节)
 * @retval         none
 */
static void parse_supercap_data(uint8_t *rx_data)
{
    // 电容电压: 字节 0-1, 0~30000 对应 0~30V
    uint16_t voltage_raw = (rx_data[0] << 8) | rx_data[1];
    supercap_data.capacitor_voltage = voltage_raw / 1000.0f;
    
    // 电容电流: 字节 2-3, -12000~12000 对应 -12~12A
    int16_t current_raw = (rx_data[2] << 8) | rx_data[3];
    supercap_data.capacitor_current = current_raw / 1000.0f;
    
    // 电池功率: 字节 4
    supercap_data.battery_power = rx_data[4];
    
    // 限制功率: 字节 5
    supercap_data.limit_power = rx_data[5];
    
    // 电容电量百分比: 字节 6
    supercap_data.capacitor_level = rx_data[6];
    
    // 状态指示: 字节 7
    supercap_data.status_indicator = rx_data[7];
    
}

