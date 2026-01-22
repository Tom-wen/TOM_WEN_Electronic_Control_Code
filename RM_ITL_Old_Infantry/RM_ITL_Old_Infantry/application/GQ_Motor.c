#include "GQ_Motor.h"
#include "CAN_receive.h"
#include "fdcan.h"
static FDCAN_TxHeaderTypeDef hoisting_tx_message;
GQ_Motor_Measure_t GQ_Motor_Measure[4];
uint32_t get_fdcan_dlc(uint16_t size);

void motor_control_current(uint8_t id, uint16_t cur)
  {
      static uint8_t tdata[] = {0x01, 0x00, 0x09, 0x05, 0x1c, 0x00, 0x00};

      memcpy(&tdata[5], &cur, sizeof(uint16_t));
			hoisting_tx_message.Identifier = 0x8000 | id;
			hoisting_tx_message.IdType = FDCAN_EXTENDED_ID;
			hoisting_tx_message.TxFrameType = FDCAN_DATA_FRAME;
			hoisting_tx_message.DataLength = 0x08;
			hoisting_tx_message.ErrorStateIndicator=FDCAN_ESI_PASSIVE;
			hoisting_tx_message.BitRateSwitch=FDCAN_BRS_OFF;
			hoisting_tx_message.FDFormat=FDCAN_CLASSIC_CAN;
			hoisting_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
			hoisting_tx_message.MessageMarker=0;
      HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3,&hoisting_tx_message, tdata);
  }

  void get_GQ_motor_measure(GQ_Motor_Measure_t *motor, uint8_t *data, uint16_t id)
  {
      motor->id = id;
      motor->position = (*(int16_t *)&data[2]) * 0.0001f;; //位置，单位：圈
      motor->velocity = (*(int16_t *)&data[4]) * 0.00025f; //速度，单位：转/秒
      motor->torque = (*(int16_t *)&data[6]) * 0.004563f; //力矩，单位：牛米
  }

  void GQ_Motor_send_current(int16_t cur_1, int16_t cur_2, int16_t cur_3, int16_t cur_4)
  { 
    motor_control_current(1, cur_1);
    motor_control_current(2, cur_2);
    motor_control_current(3, cur_3);
    motor_control_current(4, cur_4);
  }

  /**
 * @brief 读取电机位置、速度、力矩指令
 * @param id 电机ID
 */
void timed_return_motor_status(uint8_t id, int16_t t_ms)
{
      static uint8_t tdata[] = {0x05, 0xb4, 0x02, 0x00, 0x00};
      *(int16_t *)&tdata[3] = t_ms;
			hoisting_tx_message.Identifier = 0x8000 | id;
			hoisting_tx_message.IdType = FDCAN_EXTENDED_ID;
			hoisting_tx_message.TxFrameType = FDCAN_DATA_FRAME;
			hoisting_tx_message.DataLength = get_fdcan_dlc(sizeof(tdata));
			hoisting_tx_message.ErrorStateIndicator=FDCAN_ESI_PASSIVE;
			hoisting_tx_message.BitRateSwitch=FDCAN_BRS_OFF;
			hoisting_tx_message.FDFormat=FDCAN_CLASSIC_CAN;
			hoisting_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
			hoisting_tx_message.MessageMarker=0;
      HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3,&hoisting_tx_message, tdata);
}

uint32_t get_fdcan_dlc(uint16_t size)
{
    uint32_t fdcan_dlc = 0;

    if(size == 0)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_0;
    }
    else if(size <= 1)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_1;
    }
    else if(size <= 2)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_2;
    }
    else if(size <= 3)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_3;
    }
    else if(size <= 4)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_4;
    }
    else if(size <= 5)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_5;
    }
    else if(size <= 6)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_6;
    }
    else if(size <= 7)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_7;
    }
    else if(size <= 8)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_8;
    }
    else if(size <= 12)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_12;
    }
    else if(size <= 16)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_16;
    }
    else if(size <= 20)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_20;
    }
    else if(size <= 24)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_24;
    }
    else if(size <= 32)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_32;
    }
    else if(size <= 48)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_48;
    }
    else if(size <= 64)
    {
        fdcan_dlc = FDCAN_DLC_BYTES_64;
    }
    return fdcan_dlc;
}