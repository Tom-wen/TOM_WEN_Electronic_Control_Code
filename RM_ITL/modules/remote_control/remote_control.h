#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include "main.h"
#include "bsp_usart.h"
#include "Lowpass.h"
#include "stdlib.h"
#include "CRC.h"


#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/*按键宏定义*/
#define KEY_W             (rc_ctrl.key & (1 << 0))
#define KEY_S             (rc_ctrl.key & (1 << 1))
#define KEY_A             (rc_ctrl.key & (1 << 2))
#define KEY_D             (rc_ctrl.key & (1 << 3))
#define KEY_SHIFT         (rc_ctrl.key & (1 << 4))
#define KEY_CTRL          (rc_ctrl.key & (1 << 5))
#define KEY_Q             (rc_ctrl.key & (1 << 6))
#define KEY_E             (rc_ctrl.key & (1 << 7))
#define KEY_R             (rc_ctrl.key & (1 << 8))
#define KEY_F             (rc_ctrl.key & (1 << 9))
#define KEY_G             (rc_ctrl.key & (1 << 10))
#define KEY_Z             (rc_ctrl.key & (1 << 11))
#define KEY_X             (rc_ctrl.key & (1 << 12))
#define KEY_C             (rc_ctrl.key & (1 << 13))
#define KEY_V             (rc_ctrl.key & (1 << 14))
#define KEY_B             (rc_ctrl.key & (1 << 15))

#ifdef DJI_REMOTE
      //左右拨杆
      #define left_switch     rc_ctrl.rc.s[1]
      #define right_switch    rc_ctrl.rc.s[0]
      //侧拨杆
      #define side_switch     rc_ctrl.rc.ch[4]
      //侧拨杆状态
      #define side_switch_on   (side_switch >= 600)    //侧拨杆开
      #define side_switch_off  (side_switch <= -600)   //侧拨杆关

      //拨杆三种状态
      #define switch_up        ((uint16_t)1) //拨杆向上
      #define switch_mid       ((uint16_t)3) //拨杆中间
      #define switch_down      ((uint16_t)2) //拨杆向下
#endif

#ifdef FS_REMOTE
      #define left_switch       rc_ctrl.rc.s[0]   //左一拨杆
      #define left_switch2      rc_ctrl.rc.s[1]   //左二拨杆      
      #define right_switch      rc_ctrl.rc.s[2]   //右一拨杆 
      #define right_switch2     rc_ctrl.rc.s[3]   //右二拨杆
      #define left_knob         rc_ctrl.rc.ch[4]  //左旋钮
      #define right_knob        rc_ctrl.rc.ch[5]  //左旋钮
      //拨杆三种状态
      #define switch_up        ((int16_t)1) //拨杆向上
      #define switch_mid       ((int16_t)0) //拨杆中间
      #define switch_down      ((int16_t)-1) //拨杆向下
#endif

#ifdef VT_03_REMOTE
      //左右拨杆
      #define left_switch     rc_ctrl.rc.s[0]
      //侧拨杆
      #define side_switch     rc_ctrl.rc.ch[4]
      //侧拨杆状态
      #define side_switch_on   (side_switch >= 600)    //侧拨杆开
      #define side_switch_off  (side_switch <= -600)   //侧拨杆关

      //拨杆三种状态
      #define switch_up        ((uint16_t)2) //拨杆向上
      #define switch_mid       ((uint16_t)1) //拨杆中间
      #define switch_down      ((uint16_t)0) //拨杆向下
#endif

typedef __packed struct
{
        #ifdef DJI_REMOTE
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        uint8_t mouse_left:2;
        uint8_t mouse_right:2;
        uint16_t key;
        #endif
        #ifdef FS_REMOTE
        __packed struct
        {
                int16_t ch[10];         //6个通道数据
                int8_t s[4];              //四个拨杆数据
                uint8_t failsafe;       //失控标志位
                uint8_t frame_lost;     //丢帧标志位
        } rc;
        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        uint8_t mouse_left:2;
        uint8_t mouse_right:2;
        uint16_t key;
        #endif
        #ifdef VT_03_REMOTE
        uint8_t sof_1;
        uint8_t sof_2;
        __packed struct
        {
                int16_t ch[5];
                char s[1];
        } rc;
        uint64_t mode_sw:2;
        uint64_t pause:1;
        uint64_t fn_1:1;
        uint64_t fn_2:1;
        uint64_t trigger:1;

        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        uint8_t mouse_left:2;
        uint8_t mouse_right:2;
        uint8_t mouse_middle:2;
        uint16_t key;
        uint16_t crc16;

        // 边沿检测状态
        uint16_t key_last;
        uint16_t key_rising_edge;
        uint16_t key_falling_edge;        
        #endif

} RC_ctrl_t;

extern RC_ctrl_t rc_ctrl;
void remote_control_init(void);
static void sbus_to_rc(uint8_t *sbus_buf);
void vt03_to_rc(uint8_t *buf);
void KeyAccumulator_Dual_Instant(uint16_t key_inc, uint16_t key_dec,
                               float *value, float step, float limit);
int16_t rc_filter_with_deadband(LowPassFilter *lpf, int16_t raw_value, uint8_t channel);
void key_edge_update(void);

#endif
