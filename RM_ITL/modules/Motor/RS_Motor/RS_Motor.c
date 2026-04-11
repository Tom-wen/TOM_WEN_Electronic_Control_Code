#include "RS_Motor.h"

//DJI电机接收数据数组，支持多个电机
Motor_feedback RS_Motor_RX[MAX_CAN][MAX_RS_MOTORS];
// 电机ID映射表（将CAN ID映射到电机数组索引）
uint16_t rs_motor_id_map[MAX_RS_MOTORS] = {0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F};

uint16_t RS_ID_Change(uint16_t id)
{
    uint16_t rs_id = 0;
    rs_id = rs_motor_id_map[id-1];
    return rs_id;
}


/*******************************************************************************
* @功能     		: uint16_t型转float型浮点数
* @参数1            : 需要转换的值
* @参数2            : x的最小值·
* @参数3            : x的最大值
* @参数4            : 需要转换的进制数
* @返回值 			: 十进制的float型浮点数
* @概述  			: None
*******************************************************************************/
float uint16_to_float(uint16_t x,float x_min,float x_max,int bits){
    uint32_t span = (1 << bits) - 1;
		x &= span; 
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}
//灵足电机初始化
void RS_Motor_Init(MotorInstance *motors, uint8_t mode)
{
    for(int a = 0; a < motors[0].motor_count; a ++)
    {
        uint16_t can_id = RS_ID_Change(motors[a].motor_data->id);
        for(int b = 0; b < 3; b ++)
        {
            RS_Motor_Set_mode(motors[a].motor_data->hfdcan, can_id, mode);
            DWT_Delay(0.1);
        }
    }
    for(int i = 0; i < motors[0].motor_count; i ++)
    {
        uint16_t id = RS_ID_Change(motors[i].motor_data->id);
        for(int j = 0; j < 3; j++)
        {
            RS_Motor_Enable(motors[i].motor_data->hfdcan, id);
            DWT_Delay(0.1);
        }
    }
}

//灵足电机使能
void RS_Motor_Enable(FDCAN_HandleTypeDef *hfdcan, uint16_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};				//发送数据
    fdcanx_send_data(hfdcan, id, data, 8);
}

//灵足电机失能
void RS_Motor_Disable(FDCAN_HandleTypeDef *hfdcan, uint16_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};				//发送数据
    fdcanx_send_data(hfdcan, id, data, 8);
}

//灵足电机设置模式，0是MIT,1是位置模式，2是速度模式,重新上电才有效
void RS_Motor_Set_mode(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t mode)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};				//发送数据
    data[6] = mode;
    fdcanx_send_data(hfdcan, id, data, 8);
}

//灵足电机修改ID
void RS_Motor_ID_Change(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint16_t target_id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFA};				//发送数据
    data[6] = target_id;
    fdcanx_send_data(hfdcan, id, data, 8);
}

//灵足电机速度控制
void RS_Motor_Speed_mode(MotorInstance *motors)
{
    uint8_t data[8] = {0};				//发送数据
    for(int i = 0; i < motors[0].motor_count; i ++)
    {
        float speed_rad_per_s = motors[i].motor_data->target_velocity;
        float current_limit = motors[i].motor_data->target_current;
        uint16_t id = RS_ID_Change(motors[i].motor_data->id);
        if(motors[i].motor_data->motor_enable != 0)
        {
            memcpy(&data[0], &speed_rad_per_s, 4);
	        memcpy(&data[4], &current_limit, 4);
            fdcanx_send_data(motors[i].motor_data->hfdcan, ((2 << 8) | id), data, 8);
        }
        else 
        {
            data[0] = 0;
            data[1] = 0;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0;
            data[5] = 0;
            data[6] = 0;
            data[7] = 0;            
            fdcanx_send_data(motors[i].motor_data->hfdcan, ((2 << 8) | id), data, 8);
        }
        DWT_Delay(0.007);
    }
}

//灵足电机位置控制
void RS_Motor_Pos_mode(MotorInstance *motors)
{
    uint8_t data[8] = {0};				//发送数据
    for(int i = 0; i < motors[0].motor_count; i ++)
    {
        float position_rad = motors[i].motor_data->target_position;
        float speed_rad_per_s = motors[i].motor_data->target_velocity;
        uint16_t id = RS_ID_Change(motors[i].motor_data->id);
        if(motors[i].motor_data->motor_enable != 0)
        {
            memcpy(&data[0], &position_rad, 4);
	        memcpy(&data[4], &speed_rad_per_s, 4);
            fdcanx_send_data(motors[i].motor_data->hfdcan, ((1 << 8) | id), data, 8);
        }
        else 
        {
            data[0] = 0;
            data[1] = 0;
            data[2] = 0;
            data[3] = 0;
            data[4] = 0;
            data[5] = 0;
            data[6] = 0;
            data[7] = 0;            
            fdcanx_send_data(motors[i].motor_data->hfdcan, ((1 << 8) | id), data, 8);
        }
        DWT_Delay(0.007);
    }
}

//灵足电机反馈数据解析
void RS_motor_can_callback(CANRxData *Rx_data, CAN_PORT can_port)
{
    // 边界检查
    if(can_port > CAN3)
    {
      return;
    }
    // 查找电机索引
    uint8_t motor_index = 0;
    uint8_t found = 0;
    for(uint8_t i = 0; i < MAX_RS_MOTORS; i++)
    {
        if(rs_motor_id_map[i] == Rx_data->data[0])
        {
            motor_index = i;
            found = 1;
            break;
        }
    }
    // 如果找到了对应的电机ID，解析数据
    if(found)
    {
        RS_Motor_RX[can_port][motor_index].pos = uint16_to_float((Rx_data->data[1]<<8) | (Rx_data->data[2]),P_MIN,P_MAX,16);
        RS_Motor_RX[can_port][motor_index].vel = uint16_to_float((Rx_data->data[3]<<4) | (Rx_data->data[4]>>4),V_MIN,V_MAX,12);
        RS_Motor_RX[can_port][motor_index].current = uint16_to_float((Rx_data->data[4]<<8) | (Rx_data->data[5]),T_MIN,T_MAX,12);
        RS_Motor_RX[can_port][motor_index].temp = ((Rx_data->data[6]<<8) | Rx_data->data[7])*0.1;
        RS_Motor_RX[can_port][motor_index].p_int = 0.0;
        RS_Motor_RX[can_port][motor_index].v_int = 0.0;
        RS_Motor_RX[can_port][motor_index].t_int = 0.0;
        RS_Motor_RX[can_port][motor_index].tor = 0.0;
    }
}
