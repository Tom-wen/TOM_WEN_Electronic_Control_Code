#include "remote_control.h"
#include "math.h"
#include "main.h"
#include "bsp_usart.h"
#include "usart.h"
#include "detect_task.h"
#include <stdbool.h>


//遥控器控制变量
RC_ctrl_t rc_ctrl_origin;
//死区处理加滤波后的遥控器数据
RC_ctrl_t rc_ctrl;

/**
 * @brief  初始化遥控器控制模块，注册回调函数
 */
void remote_control_init(void)
{
    #ifdef DT7_rc_ctrl
    bsp_usart5_set_callback(sbus_to_rc);//开启回调处理函数
    #endif

    #ifdef i6x_rc_ctrl
    bsp_usart5_set_callback(sbus_to_i6x);//开启回调处理函数
    #endif

}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl_origin: 遥控器数据指
  * @retval         none
  */
 static void sbus_to_rc(uint8_t *sbus_buf);
static void sbus_to_rc(uint8_t *sbus_buf)
{
    if (sbus_buf == NULL)
    {
        return;
    }
	
    rc_ctrl.rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl.rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl.rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl.rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl.rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl.rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl.mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl.mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl.mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl.mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl.mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl.key = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl.rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL
    rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;
    detect_hook(DBUS_TOE);
}






//i6x遥控器相关定义
#define TO_STICK(v)  (((v) < 0) - ((v) > 0))
//摇杆通道值映射宏定义开关
#define MAPPING_ENABLE 1

i6x_ctrl_t i6x_ctrl;

static int16_t map_to_660(const int16_t val) 
{
    if (val >= 0)
        return (int16_t) floorf((660.0f / 783.0f) * (float) val + 0.5f);
    else
        return (int16_t) floorf((660.0f / 784.0f) * (float) val + 0.5f);
}
void sbus_to_i6x( uint8_t *sbus_data) 
{
    if (sbus_data[0] != 0x0F || sbus_data[24] != 0x00) 
    {
        return;
    }
    i6x_ctrl.ch[0] = (int16_t) (((sbus_data[1] | (sbus_data[2] << 8)) & 
    0x07FF) - 1024);
    i6x_ctrl.ch[1] = (int16_t) ((((sbus_data[2] >> 3) | (sbus_data[3] << 5)) 
    & 0x07FF) - 1024);
    i6x_ctrl.ch[2] = (int16_t) ((((sbus_data[3] >> 6) | (sbus_data[4] << 2) | 
    (sbus_data[5] << 10)) & 0x07FF) - 1024);
    i6x_ctrl.ch[3] = (int16_t) ((((sbus_data[5] >> 1) | (sbus_data[6] << 7)) 
    & 0x07FF) - 1024);
    i6x_ctrl.ch[4] = (int16_t) ((((sbus_data[6] >> 4) | (sbus_data[7] << 4)) 
    & 0x07FF) - 1024);
    i6x_ctrl.ch[5] = (int16_t) ((((sbus_data[7] >> 7) | (sbus_data[8] << 1) | 
    (sbus_data[9] << 9)) & 0x07FF) - 1024);
    i6x_ctrl.s[0] = (int8_t) TO_STICK((((sbus_data[9] >> 2) | (sbus_data[10] 
    << 6)) & 0x07FF) - 1024);
    i6x_ctrl.s[1] = (int8_t) TO_STICK((((sbus_data[10] >> 5) | (sbus_data[11] 
    << 3)) & 0x07FF) - 1024);
    i6x_ctrl.s[2] = (int8_t) TO_STICK(((sbus_data[12] | (sbus_data[13] << 8)) 
    & 0x07FF) - 1024);
    i6x_ctrl.s[3] = (int8_t) TO_STICK((((sbus_data[13] >> 3) | (sbus_data[14] 
    << 5)) & 0x07FF) - 1024);
    
    #if MAPPING_ENABLE
        for (int i = 0; i < 6; i++)
        {
            i6x_ctrl.ch[i] = map_to_660(i6x_ctrl.ch[i]);
        }
        const uint8_t flag = sbus_data[23];
        i6x_ctrl.frame_lost = (flag >> 2) & 0x01;
        i6x_ctrl.failsafe = (flag >> 3) & 0x01;
        if (i6x_ctrl.frame_lost==0)
        {
            detect_hook(DBUS_TOE);
        }
}
#endif








//图传遥控器相关定义


static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);

static uint16_t crc16_init = 0xffff;
static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
 * @brief Get the crc16 checksum
 *
 * @param p_msg Data to check
 * @param lenData length
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if(p_msg == NULL)
    {
        return 0xffff;
    }

    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}

/**
 * @brief crc16 verify function
 *
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if((p_msg == NULL) || (len <= 2))
    {
        return false;
    }
    w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}



vtm_rc_data_t vtm_rc_data;
/**
 * @brief 解析裁判系统图传模块（VTM）数据帧
 * @param vtm_buf 21字节的数据帧缓冲区
 * @param rc_data 输出的遥控数据结构体
 */
void vtm_data_parse(uint8_t *vtm_buf)
{
    // 1. 检查指针有效性
    if (vtm_buf == NULL) 
    {
        return;
    }
    
    // 2. 检查帧头
    if (vtm_buf[0] != 0xA9 || vtm_buf[1] != 0x53) 
    {
        return;
    }
    
    // 3. 校验CRC（使用您提供的verify_crc16_check_sum函数）
    if (!verify_crc16_check_sum(vtm_buf, 21)) 
    {
        return;
    }

    // 4. 构造64位位域数据 (字节2-9，小端序)
    uint64_t bit_field = 0;
    for (int i = 0; i < 8; i++) 
    {
        bit_field |= ((uint64_t)vtm_buf[2 + i]) << (i * 8);
    }

    // 5. 解析通道数据（每个通道11位，连续存储）
    // ch_0: bit[0:10], ch_1: bit[11:21], ch_2: bit[22:32], ch_3: bit[33:43]
    vtm_rc_data.ch[0] = (int16_t)((int32_t)((bit_field >> 0) & 0x7FF) - 1024);  // ch_0
    vtm_rc_data.ch[1] = (int16_t)((int32_t)((bit_field >> 11) & 0x7FF) - 1024); // ch_1
    vtm_rc_data.ch[2] = (int16_t)((int32_t)((bit_field >> 22) & 0x7FF) - 1024); // ch_2
    vtm_rc_data.ch[3] = (int16_t)((int32_t)((bit_field >> 33) & 0x7FF) - 1024); // ch_3


    // 拨轮(11位): bit[49:59]
    vtm_rc_data.wheel = (int16_t)((bit_field >> 49) & 0x7FF) - 1024;

    // 摇杆映射（根据说明书示例）
    vtm_rc_data.right_x = vtm_rc_data.ch[0];  // 右摇杆水平
    vtm_rc_data.right_y = vtm_rc_data.ch[1];  // 右摇杆垂直
    vtm_rc_data.left_y  = vtm_rc_data.ch[2];  // 左摇杆垂直
    vtm_rc_data.left_x  = vtm_rc_data.ch[3];  // 左摇杆水平

    // 6. 解析开关和按钮
    // mode_sw: 2位，bit[44:45]
    vtm_rc_data.mode_sw = (uint8_t)((bit_field >> 44) & 0x03);

    // pause: 1位，bit[46]
    vtm_rc_data.pause = (uint8_t)((bit_field >> 46) & 0x01);

    // fn_1: 1位，bit[47]
    vtm_rc_data.fn_1 = (uint8_t)((bit_field >> 47) & 0x01);

    // fn_2: 1位，bit[48]
    vtm_rc_data.fn_2 = (uint8_t)((bit_field >> 48) & 0x01);

    // trigger: 1位，bit[60]
    vtm_rc_data.trigger = (uint8_t)((bit_field >> 60) & 0x01);

    // 7. 解析鼠标数据（从Byte 10开始）
    vtm_rc_data.mouse_x = (int16_t)(vtm_buf[10] | (vtm_buf[11] << 8));  // 鼠标X轴
    vtm_rc_data.mouse_y = (int16_t)(vtm_buf[12] | (vtm_buf[13] << 8));  // 鼠标Y轴
    vtm_rc_data.mouse_z = (int16_t)(vtm_buf[14] | (vtm_buf[15] << 8));  // 鼠标Z轴

    // 鼠标按键（各2位）
    vtm_rc_data.mouse_left   = (vtm_buf[16] >> 6) & 0x03;  // 左键
    vtm_rc_data.mouse_right  = (vtm_buf[16] >> 4) & 0x03;  // 右键
    vtm_rc_data.mouse_middle = (vtm_buf[16] >> 2) & 0x03;  // 中键

    // 8. 解析键盘数据（2字节）
    vtm_rc_data.key = (uint16_t)(vtm_buf[17] | (vtm_buf[18] << 8));

    // 9. 调用检测钩子函数
    detect_hook(DBUS_TOE);

    return;
}