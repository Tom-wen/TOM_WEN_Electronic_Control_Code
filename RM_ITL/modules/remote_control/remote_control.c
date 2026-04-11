#include "remote_control.h"

//遥控器控制变量
RC_ctrl_t rc_ctrl_origin;
//死区处理加滤波后的遥控器数据
RC_ctrl_t rc_ctrl;

/**
 * @brief  初始化遥控器控制模块，注册回调函数
 */
void remote_control_init(void)
{
    #if defined(DJI_REMOTE) || defined(FS_REMOTE)
        bsp_usart5_set_callback(sbus_to_rc);//开启回调处理函数
    #endif
    #ifdef VT_03_REMOTE
        bsp_usart7_set_callback(vt03_to_rc);
    #endif 
}

#ifdef DJI_REMOTE
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
    rc_ctrl.mouse_x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl.mouse_y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl.mouse_z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl.mouse_left = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl.mouse_right = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl.key = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl.rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL
    rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
#endif 

#ifdef FS_REMOTE

//拨杆s[0] ~ s[3]三段式归⼀化为 1 / 0 / -1 
// 想要对调-1和1只需要把括号⾥的< 和 >对调⼀下
#define TO_STICK(v)  (((v) < 0) - ((v) > 0))
//摇杆通道值映射宏定义开关
#define MAPPING_ENABLE  1
static int16_t map_to_660(const int16_t val)
{
    if (val >= 0){
        return (int16_t) floorf((660.0f / 783.0f) * (float) val + 0.5f);
    }
    else{
        return (int16_t) floorf((660.0f / 784.0f) * (float) val + 0.5f);
    }
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
	rc_ctrl.rc.ch[0] =  (int16_t) (((sbus_buf[1] | (sbus_buf[2] << 8)) & 0x07FF) - 1024);
    rc_ctrl.rc.ch[1] =  (int16_t) ((((sbus_buf[2] >> 3) | (sbus_buf[3] << 5)) & 0x07FF) - 1024);
    rc_ctrl.rc.ch[3] =  (int16_t) ((((sbus_buf[3] >> 6) | (sbus_buf[4] << 2) | (sbus_buf[5] << 10)) & 0x07FF) - 1024);
    rc_ctrl.rc.ch[2] =  (int16_t) ((((sbus_buf[5] >> 1) | (sbus_buf[6] << 7)) & 0x07FF) - 1024);
    rc_ctrl.rc.ch[4] =  (int16_t) ((((sbus_buf[6] >> 4) | (sbus_buf[7] << 4)) & 0x07FF) - 1024);
    rc_ctrl.rc.ch[5] =  (int16_t) ((((sbus_buf[7] >> 7) | (sbus_buf[8] << 1) | (sbus_buf[9] << 9)) & 0x07FF) - 1024);
    rc_ctrl.rc.s[0] = (int8_t) TO_STICK((((sbus_buf[9] >> 2) | (sbus_buf[10] << 6)) & 0x07FF) - 1024);
    rc_ctrl.rc.s[1] = (int8_t) TO_STICK((((sbus_buf[10] >> 5) | (sbus_buf[11] << 3)) & 0x07FF) - 1024);
    rc_ctrl.rc.s[2] = (int8_t) TO_STICK(((sbus_buf[12] | (sbus_buf[13] << 8)) & 0x07FF) - 1024);
    rc_ctrl.rc.s[3] = (int8_t) TO_STICK((((sbus_buf[13] >> 3) | (sbus_buf[14] << 5)) & 0x07FF) - 1024);

    //通道值映射
 
    #if MAPPING_ENABLE
    for (int i = 0; i < 6; i++)
    {
        rc_ctrl.rc.ch[i] = map_to_660(rc_ctrl.rc.ch[i]);
    }
    //失控丢帧标志位，遥控器断连后先后置1 
    const uint8_t flag = sbus_buf[23];
    rc_ctrl.rc.frame_lost = (flag >> 2) & 0x01;
    rc_ctrl.rc.failsafe = (flag >> 3) & 0x01;
    #endif
}

#endif

#ifdef VT_03_REMOTE

//图传遥控器数据接收
void vt03_to_rc(uint8_t *buf)
{
    // 检查帧头 0xA9 0x53
    if (buf[0] != 0xA9 || buf[1] != 0x53)
    {
        return;
    }

    // 用原有CRC校验函数，直接返回真假
    if (!verify_CRC16_check_sum(buf, 21))
    {
        return; // CRC校验失败
    }

    // ========== 解析数据 (CRC已通过) ==========
    
    // 构造64位位域数据 (字节2-9，小端序)
    uint64_t bit_field = 0;
    for (int i = 0; i < 8; i++) 
    {
        bit_field |= ((uint64_t)buf[2 + i]) << (i * 8);
    }

    // 通道0~3：偏移16,27,38,49 (各11位) → 相对bit_field位0,11,22,33
    // 映射到 [-660, 660]，中间值为0
    rc_ctrl.rc.ch[0] =  (int16_t)((bit_field >> 0)  & 0x7FF) - 1024;
    rc_ctrl.rc.ch[1] =  (int16_t)((bit_field >> 11) & 0x7FF) - 1024;
    rc_ctrl.rc.ch[3] =  (int16_t)((bit_field >> 22) & 0x7FF) - 1024;
    rc_ctrl.rc.ch[2] =  (int16_t)((bit_field >> 33) & 0x7FF) - 1024;
    // 拨轮(11位):偏移65→位49，映射到 [-660, 660]，中间值为0
    rc_ctrl.rc.ch[4] =  (int16_t)((bit_field >> 49) & 0x7FF) - 1024;

    // 挡位开关(2位):偏移60→位44
    rc_ctrl.rc.s[0] = (uint8_t)((bit_field >> 44) & 0x03);
    // 暂停键(1位):偏移62→位46
    rc_ctrl.pause = (uint8_t)((bit_field >> 46) & 0x01);

    // 自定义键左(1位):偏移63→位47
    rc_ctrl.fn_1 = (uint8_t)((bit_field >> 47) & 0x01);

    // 自定义键右(1位):偏移64→位48
    rc_ctrl.fn_2 = (uint8_t)((bit_field >> 48) & 0x01);


    // 扳机键(1位):偏移76→位60
    rc_ctrl.trigger = (uint8_t)((bit_field >> 60) & 0x01);

    // 鼠标X轴(16位有符号):偏移80位=字节10-11，小端序
    rc_ctrl.mouse_x = (int16_t)(buf[10] | (buf[11] << 8));

    // 鼠标Y轴:偏移96位=字节12-13
    rc_ctrl.mouse_y = (int16_t)(buf[12] | (buf[13] << 8));

    // 鼠标Z轴:偏移112位=字节14-15
    rc_ctrl.mouse_z = (int16_t)(buf[14] | (buf[15] << 8));

    // 鼠标按键:偏移128位=字节16 (位6-7左键,位4-5右键,位2-3中键)
    rc_ctrl.mouse_left   = (buf[16] >> 0) & 0x03;
    rc_ctrl.mouse_right  = (buf[16] >> 2) & 0x03;
    rc_ctrl.mouse_middle = (buf[16] >> 4) & 0x03;

    // 键盘(16位):偏移136位=字节17-18，小端序
    rc_ctrl.key = (uint16_t)(buf[17] | (buf[18] << 8));

    // 保存帧头和CRC
    rc_ctrl.sof_1 = buf[0];
    rc_ctrl.sof_2 = buf[1];
    rc_ctrl.crc16 = (uint16_t)(buf[19] | (buf[20] << 8));
}

// 更新函数
void key_edge_update(void)
{
    uint16_t key_now = rc_ctrl.key;
    rc_ctrl.key_last = 0;
    rc_ctrl.key_rising_edge  = key_now & (~rc_ctrl.key_last);
    rc_ctrl.key_falling_edge = (~key_now) & rc_ctrl.key_last;
    rc_ctrl.key_last = key_now;
}

#endif

/**
 * @brief 双键控制的累加值处理
 * @param key_inc 增加按键（正方向）的位掩码
 * @param key_dec 减小按键（负方向）的位掩码
 * @param value 当前值的指针
 * @param step 每次增加/减小的步长
 * @param decay 衰减速率（按键松开时）
 * @param limit 限幅值（最大正值为limit，最小负值为-limit）
 * @retval 无
 * 
 * @details 
 * - key_inc 按下：value 逐渐增加，最大到 limit
 * - key_dec 按下：value 逐渐减小，最小到 -limit
 * - 两个键都松开：value 衰减到 0
 */
void KeyAccumulator_Dual_Instant(uint16_t key_inc, uint16_t key_dec,
                               float *value, float step, float limit)
{
    if (rc_ctrl.key & key_inc)
    {
        *value += step;
        if (*value > limit) *value = limit;
    }
    else if (rc_ctrl.key & key_dec)
    {
        *value -= step;
        if (*value < -limit) *value = -limit;
    }
    else
    {
        *value = 0;  // 直接归零，无衰减
    }
}

// 死区处理加滤波器
int16_t rc_filter_with_deadband(LowPassFilter *lpf, int16_t raw_value, uint8_t channel)
{
    #define RC_DEADBAND 10
    static int16_t last_raw_value[5] = {0};  // 5个通道各自独立
    // 死区处理
    if(abs(raw_value) <= RC_DEADBAND)
    {
        raw_value = 0;
    }
    if(abs(raw_value - last_raw_value[channel]) <= 100)
    {
        raw_value = last_raw_value[channel];
    }
    last_raw_value[channel] = raw_value;
    // 低通滤波
    float filtered = LowPassFilter_operator(lpf, (float)raw_value);
    return (int16_t)filtered;
}


