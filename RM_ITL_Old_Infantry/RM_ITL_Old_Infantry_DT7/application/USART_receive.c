#include "USART_receive.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "main.h"
#include "detect_task.h"
#include "usart.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "stdint.h"
#include "usbd_cdc_if.h"
#include "INS_task.h"
#include "user_lib.h"
#include "referee_usart_task.h"

extern gimbal_control_t gimbal_control;
auto_shoot_t auto_shoot;

/**
 * @brief 用户数据解析函数
 *
 * @param buf 待解析数据的缓冲区指针
 * @param auto_shoot 自动射击数据结构指针
 */
void user_data_solve(volatile const uint8_t *buf, auto_shoot_t *auto_shoot)
{
    // 检查帧头和帧尾
    if (buf[0] == 'V' && buf[1] == 'G' && buf[27] == 'V')
    {
        // 射击模式
        auto_shoot->mode = buf[2];

        // 解析float数据 (协议为小端序)
        // yaw角度
        memcpy(&auto_shoot->yaw_add, &buf[3], sizeof(float));
        // yaw角速度
        memcpy(&auto_shoot->yaw_vel, &buf[7], sizeof(float));
        // yaw角加速度
        memcpy(&auto_shoot->yaw_acc, &buf[11], sizeof(float));
        // pitch角度
        memcpy(&auto_shoot->pitch_add, &buf[15], sizeof(float));
        auto_shoot->pitch_add = -auto_shoot->pitch_add; // 协议中pitch角度为正向向下，转换为正向向上
        // pitch角速度
        memcpy(&auto_shoot->pitch_vel, &buf[19], sizeof(float));
        // pitch角加速度
        memcpy(&auto_shoot->pitch_acc, &buf[23], sizeof(float));

        detect_hook(USER_USART_DATA_TOE);
    }

}

void send_vision_data(void)
{
    uint8_t sendBuffer[256]; // 确保足够大
    uint16_t offset = 0;

    // 帧头
    sendBuffer[offset++] = 'G';
    sendBuffer[offset++] = 'V';

    // mode
    sendBuffer[offset++] = 1; // 自动模式

    float q[4];
    
    float cy = cos(INS_data.angle_yaw * 0.5f);
    float sy = sin(INS_data.angle_yaw * 0.5f);
    float cp = cos(INS_data.angle_roll * 0.5f);
    float sp = sin(INS_data.angle_roll * 0.5f);
    float cr = cos(INS_data.angle_pitch * 0.5f);
    float sr = sin(INS_data.angle_pitch * 0.5f);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = cy * sp * cr + sy * cp * sr;
    q[3] = sy * cp * cr - cy * sp * sr;

    float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-6f) 
    { // 避免除零
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }


    //四元数 (使用从yaw和pitch计算出的四元数)
    memcpy(&sendBuffer[offset], &q[0], 4);
    offset += 4;
    memcpy(&sendBuffer[offset], &q[1], 4);
    offset += 4;
    memcpy(&sendBuffer[offset], &q[2], 4);
    offset += 4;
    memcpy(&sendBuffer[offset], &q[3], 4);
    offset += 4;

    // yaw (4字节)
    memcpy(&sendBuffer[offset], &INS_data.angle_yaw, 4);
    offset += 4;

    // yaw角速度 (4字节)
    memcpy(&sendBuffer[offset], &INS_data.ax, 4);
    offset += 4;

    // pitch (4字节)
    float pitch_send = -INS_data.angle_pitch;
    memcpy(&sendBuffer[offset], &pitch_send, 4);
    offset += 4;

    // pitch角速度 (4字节)
    memcpy(&sendBuffer[offset], &INS_data.ay, 4);
    offset += 4;

    // 子弹速度 (4字节)
    memcpy(&sendBuffer[offset], &referee_data.shoot_data.initial_speed, 4);
    offset += 4;

    // 子弹计数 (2字节)
    memcpy(&sendBuffer[offset], 0, 2);
    offset += 2;

    // 帧尾
    sendBuffer[offset++] = 'G';

    // 发送
    //usart_tx_dma_send(&huart7, sendBuffer, offset);
    CDC_Transmit_HS(sendBuffer, offset);
}
