#include "DM_Motor.h"

//电机参数
motor_t MOTOR;
//电机接收数据数组
Motor_feedback DM_Motor_RX[MAX_CAN][MAX_DM_MOTORS];
// 电机ID映射表（将CAN ID映射到电机数组索引）
uint16_t dm_motor_id_map[MAX_DM_MOTORS] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18};
/**
 * 将浮点数转换为无符号整数，基于给定的范围和位数。
 *
 * 参数:
 *   x_float: 要转换的浮点数值
 *   x_min:   输入范围的最小值
 *   x_max:   输入范围的最大值
 *   bits:    目标整数的位数（必须大于等于1，且bits位数对应的范围不超过整型限制）
 *
 * 返回值:
 *   转换后的无符号整数值，范围在0到(1<<bits)-1之间。若输入超出范围可能导致结果溢出。
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
 * 将无符号整数转换为指定范围内的浮点数
 * 
 * 参数说明：
 * x_int  - 输入的无符号整数（0 <= x_int < (1<<bits)）
 * x_min  - 输出浮点数的最小值（对应x_int=0时的输出）
 * x_max  - 输出浮点数的最大值（对应x_int=(1<<bits)-1时的输出）
 * bits   - 量化使用的位数（决定输入整数的精度）
 * 
 * 返回值：
 * 转换后的浮点数，均匀分布在[x_min, x_max]区间内
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
//达妙电机初始化
void DM_Motor_Init(MotorInstance *motors)
{
    MOTOR.PMAX = 3.1415926f;//上位机设置的，单位为rad
    MOTOR.TMAX = 10.0f;
    MOTOR.VMAX = 30.0f;
    
    //初始化电机接收数据数组
    for(int i = 0; i < MAX_CAN; i++)
    {
        for(int j = 0; j < MAX_DM_MOTORS; j++) 
        {
            DM_Motor_RX[i][j].pos = 0.0f;
            DM_Motor_RX[i][j].vel = 0.0f;
            DM_Motor_RX[i][j].tor = 0.0f;
            DM_Motor_RX[i][j].temp = 0.0f;
            DM_Motor_RX[i][j].current = 0.0f;
        }
    }
    
    //write_save_data(&hfdcan2, 1, 10, mit_mode, 0, 0, 0);//电机写入模式，此时为位置速度模式
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        for(int j = 0; j < 5; j++)
        {
            motor_enable(motors[i].motor_data->hfdcan, motors[i].motor_data->id);//电机使能
        }
    }
    //DM_Pos_mode(&hfdcan1, 1, 0, 5);//电机上电回零位
}
//达妙电机使能
void motor_enable(FDCAN_HandleTypeDef *hfdcan, uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    fdcanx_send_data(hfdcan, id, data, 8);
    DWT_Delay(0.3);
}
//达妙电机失能
void motor_disable(FDCAN_HandleTypeDef *hfdcan, uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

    fdcanx_send_data(hfdcan, id, data, 8);
}

//达妙电机使能
void DM_Motor_enable(MotorControlData *motors)
{
    motors->motor_enable = 1;
}

//达妙电机失能（标志位）
void DM_Motor_disable(MotorControlData *motors)
{
    motors->motor_enable = 0;
}

/**
 * @brief 发送MIT模式控制指令到指定CAN节点
 * 
 * 该函数将位置、速度、力控参数打包为CAN帧格式并发送，用于控制支持MIT协议的电机驱动器。
 * 参数范围需符合电机驱动器规格，超出范围的值会被裁剪。
 * 
 * @param hfdcan FDCAN句柄指针
 * @param id CAN节点ID（0x00-0x0F）
 * @param pos 目标位置(rad)，范围[-MOTOR.PMAX, MOTOR.PMAX]
 * @param vel 目标速度(rad/s)，范围[-MOTOR.VMAX, MOTOR.VMAX]
 * @param kp 位置环增益，范围[KP_MIN, KP_MAX]
 * @param kd 微分增益，范围[KD_MIN, KD_MAX]
 * @param tor 输出扭矩限制(Nm)，范围[-MOTOR.TMAX, MOTOR.TMAX]
 * 
 * @return 无
 */
void DM_Mit_mode(MotorInstance *motors)
{
    uint8_t data[8];
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        if(motors[i].motor_data->motor_enable != 0)
        {
            pos_tmp = float_to_uint(motors[i].motor_data->pos, -MOTOR.PMAX, MOTOR.PMAX, 16);
            vel_tmp = float_to_uint(motors[i].motor_data->vel, -MOTOR.VMAX, MOTOR.VMAX, 12);
            tor_tmp = float_to_uint(motors[i].motor_data->tor, -MOTOR.TMAX, MOTOR.TMAX, 12);
            kp_tmp  = float_to_uint(motors[i].motor_data->kp,  KP_MIN, KP_MAX, 12);
            kd_tmp  = float_to_uint(motors[i].motor_data->kd,  KD_MIN, KD_MAX, 12);

            data[0] = (pos_tmp >> 8);
            data[1] = pos_tmp;
            data[2] = (vel_tmp >> 4);
            data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
            data[4] = kp_tmp;
            data[5] = (kd_tmp >> 4);
            data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
            data[7] = tor_tmp;

            fdcanx_send_data(motors[i].motor_data->hfdcan, 0x00 | motors[i].motor_data->id, data, 8);
        }
        else
        {
            data[0] = 0x7F;
            data[1] = 0xFF;
            data[2] = 0x7F;
            data[3] = 0xF0;
            data[4] = 0x00;
            data[5] = 0x00;
            data[6] = 0x07;
            data[7] = 0xFF;

            fdcanx_send_data(motors[i].motor_data->hfdcan, 0x00 | motors[i].motor_data->id, data, 8);
        }
        DWT_Delay(0.0005);
    }
}
/**
 * @brief  通过FDCAN总线发送位置和速度控制模式数据
 * @param  hfdcan：FDCAN外设句柄指针
 * @param  id：CAN消息标识符低4位（扩展帧）
 * @param  pos：位置目标值（float类型32位存储格式）单位rad
 * @param  vel：速度目标值（float类型32位存储格式）单位rad/s
 * @retval 无
 */
void DM_Pos_mode(MotorInstance *motors)
{
    uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	for(int i = 0; i < motors[0].motor_count; i++)
    {
        if(motors[i].motor_data->motor_enable != 0)
        {
            pbuf=(uint8_t*)&motors[i].motor_data->pos;
	        vbuf=(uint8_t*)&motors[i].motor_data->vel;
            
	        data[0] = *pbuf;
	        data[1] = *(pbuf+1);
	        data[2] = *(pbuf+2);
	        data[3] = *(pbuf+3);

	        data[4] = *vbuf;
	        data[5] = *(vbuf+1);
	        data[6] = *(vbuf+2);
	        data[7] = *(vbuf+3);

            fdcanx_send_data(motors[i].motor_data->hfdcan, 0x100 | motors[i].motor_data->id, data, 8);
        }
        else
        {
            data[0] = 0x00;
            data[1] = 0x00;
            data[2] = 0x00;
            data[3] = 0x00;
            data[4] = 0x00;
            data[5] = 0x00;
            data[6] = 0x00;
            data[7] = 0x00;

            fdcanx_send_data(motors[i].motor_data->hfdcan, 0x100 | motors[i].motor_data->id, data, 8);
        }
        DWT_Delay(0.0005);
    }
}
/**
 * @brief  发送速度模式指令到指定的FDCAN节点
 * @param  hfdcan FDCAN外设句柄指针
 * @param  id 目标节点ID（0-255）
 * @param  vel 浮点型速度值（单位由应用层定义）单位rad/s
 * @retval None
 */
void DM_Spd_mode(MotorInstance *motors)
{
    uint8_t *vbuf;
    uint8_t data[4];
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        if(motors[i].motor_data->motor_enable != 0)
        {
            vbuf=(uint8_t*)&motors[i].motor_data->vel;
            
            data[0] = *vbuf;
            data[1] = *(vbuf+1);
            data[2] = *(vbuf+2);
            data[3] = *(vbuf+3);

            fdcanx_send_data(motors[i].motor_data->hfdcan, 0x200 | motors[i].motor_data->id, data, 4);
        }
        else
        {
            data[0] = 0x00;
            data[1] = 0x00;
            data[2] = 0x00;
            data[3] = 0x00;

            fdcanx_send_data(motors[i].motor_data->hfdcan, 0x200 | motors[i].motor_data->id, data, 4);
        }
        DWT_Delay(0.0005);
    }
}
/**
  * @brief  通过CAN总线发送指定ID设备的力位混控模式数据
  * @param  hfdcan: FDCAN句柄指针
  * @param  id: 设备ID（0-127）
  * @param  pos: 位置值（float类型）
  * @param  vel: 速度值（float类型，单位0.01 LSB）
  * @param  cur: 电流值（float类型，单位0.0001 LSB）
  * @retval 无
  */
void DM_Psi_mode(MotorInstance *motors)
{
    uint8_t *pbuf, *vbuf, *ibuf;
    uint8_t data[8];
    for(int i = 0; i < motors[0].motor_count; i++)
    {
        if(motors[i].motor_data->motor_enable != 0)
        {
            uint16_t u16_vel = motors[i].motor_data->vel*100;
            uint16_t u16_cur = motors[i].motor_data->cur*10000;

            pbuf=(uint8_t*)&motors[i].motor_data->pos;
            vbuf=(uint8_t*)&u16_vel;
            ibuf=(uint8_t*)&u16_cur;

            data[0] = *pbuf;
            data[1] = *(pbuf+1);
            data[2] = *(pbuf+2);
            data[3] = *(pbuf+3);

            data[4] = *vbuf;
            data[5] = *(vbuf+1);

            data[6] = *ibuf;
            data[7] = *(ibuf+1);

            fdcanx_send_data(motors[i].motor_data->hfdcan, 0x300 | motors[i].motor_data->id, data, 8);
        }
        else
        {
            data[0] = 0x00;
            data[1] = 0x00;
            data[2] = 0x00;
            data[3] = 0x00;
            data[4] = 0x00;
            data[5] = 0x00;
            data[6] = 0x00;
            data[7] = 0x00;

            fdcanx_send_data(motors[i].motor_data->hfdcan, 0x300 | motors[i].motor_data->id, data, 8);
        }
        DWT_Delay(0.0005);
    }

}

//读取电机反馈数据
void read_motor_rx(FDCAN_HandleTypeDef *hfdcan, uint16_t id)
{
    uint8_t can_id_l = id & 0x0F;
    uint8_t can_id_h = (id >> 4) & 0x0F;
    uint8_t data[4] = {can_id_l, can_id_h, 0xCC, 0x00};
    fdcanx_send_data(hfdcan, 0x7FF, data, 4);
}

// 发送读取电机寄存器命令
void read_motor_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t rid) 
{
    uint8_t can_id_l = id & 0x0F;
    uint8_t can_id_h = (id >> 4) & 0x0F;
    uint8_t data[4] = {can_id_l, can_id_h, 0x33, rid};
    fdcanx_send_data(hfdcan, 0x7FF, data, 4);
}

// 发送写入电机寄存器命令
void write_motor_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    uint8_t can_id_l = id & 0xFF;       // 低 8 位
    uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位
    uint8_t data[8] = {can_id_l, can_id_h, 0x55, rid, d0, d1, d2, d3};
    fdcanx_send_data(hfdcan, 0x7FF, data, 8);
}

// 发送保存电机参数命令
void save_motor_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t rid) 
{
    uint8_t can_id_l = id & 0xFF;       // 低 8 位
    uint8_t can_id_h = (id >> 8) & 0x07; // 高 3 位
    uint8_t data[4] = {can_id_l, can_id_h, 0xAA, 0x01};
    fdcanx_send_data(hfdcan, 0x7FF, data, 4);
}

//写入并保存数据
//rid为10时改变电机模式
void write_save_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t rid, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    write_motor_data(hfdcan, id, rid, d0, d1, d2, d3);
    HAL_Delay(100);
    save_motor_data(hfdcan, id, rid);
}

//4310使能判断
void Motor4310_enable_judge(MotorInstance *motors)
{
    if(motors[0].motor_data->feedback->pos == 0){
        DM_Motor_Init(motors);
    }
}

//DM电机数据处理
void DM_motor_can_callback(CANRxData *Rx_data, CAN_PORT can_port)
{
    // 边界检查
    if(can_port > CAN3)
    {
      return;
    }
    // 查找电机索引
    uint8_t motor_index = 0;
    uint8_t found = 0;
    for(uint8_t i = 0; i < MAX_DM_MOTORS; i++)
    {
        if(dm_motor_id_map[i] == Rx_data->id)
        {
            motor_index = i;
            found = 1;
            break;
        }
    }
    if(found)
    {
        // 解析数据到二维数组
        DM_Motor_RX[can_port][motor_index].p_int = (Rx_data->data[1] << 8) | Rx_data->data[2];
        DM_Motor_RX[can_port][motor_index].v_int = (Rx_data->data[3] << 4) | (Rx_data->data[4] >> 4);
        DM_Motor_RX[can_port][motor_index].t_int = ((Rx_data->data[4] & 0xF) << 8) | Rx_data->data[5];
        DM_Motor_RX[can_port][motor_index].pos = uint_to_float(DM_Motor_RX[can_port][motor_index].p_int, -MOTOR.PMAX, MOTOR.PMAX, 16);
        DM_Motor_RX[can_port][motor_index].vel = uint_to_float(DM_Motor_RX[can_port][motor_index].v_int, -MOTOR.VMAX, MOTOR.VMAX, 12);
        DM_Motor_RX[can_port][motor_index].tor = uint_to_float(DM_Motor_RX[can_port][motor_index].t_int, -MOTOR.TMAX, MOTOR.TMAX, 12);
        DM_Motor_RX[can_port][motor_index].temp = (float)(Rx_data->data[6]);
        DM_Motor_RX[can_port][motor_index].current = 0.0;
    }

}