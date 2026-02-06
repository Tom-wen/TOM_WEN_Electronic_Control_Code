#include "remote_control.h"
#include "math.h"
#include "detect_task.h"

//遥控器控制变量
RC_ctrl_t rc_ctrl_origin;
//死区处理加滤波后的遥控器数据
RC_ctrl_t rc_ctrl;

/**
 * @brief  初始化遥控器控制模块，注册回调函数
 */
void remote_control_init(void)
{
    bsp_usart5_set_callback(sbus_to_rc);//开启回调处理函数
}

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl_origin: 遥控器数据指
  * @retval         none
  */
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





#define TO_STICK(v)  (((v) < 0) - ((v) > 0))
//i6x遥控器相关定义
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