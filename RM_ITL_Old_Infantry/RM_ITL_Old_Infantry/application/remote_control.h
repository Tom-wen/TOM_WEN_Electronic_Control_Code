#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include "main.h"
#include "bsp_usart.h"
#include "stdlib.h"

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)
//²¦¸ËÈýÖÖ×´Ì¬
#define switch_up        ((uint16_t)1) //²¦¸ËÏòÉÏ
#define switch_mid       ((uint16_t)3) //²¦¸ËÖÐ¼ä
#define switch_down        ((uint16_t)2) //²¦¸ËÏòÏÂ

//×óÓÒ²¦¸Ë
#define left_switch     rc_ctrl.rc.s[1]
#define right_switch    rc_ctrl.rc.s[0]

//²à²¦¸Ë
#define side_switch     rc_ctrl.rc.ch[4]
//²à²¦¸Ë×´Ì¬
#define side_switch_on   (side_switch >= 600)    //²à²¦¸Ë¿ª
#define side_switch_off  (side_switch <= -600)   //²à²¦¸Ë¹Ø

typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        uint16_t key;
} RC_ctrl_t;

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u


/* ----------------------- RC Switch Definition----------------------------- */
#define LEFT_SWITCH 1
#define RIGHT_SWITCH 0
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)

#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)//¿ªÄ¦²ÁÂÖ
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)//¹Ø±ÕÄ¦²ÁÂÖ
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)//×ó×ª90¡ã
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)//ÓÒ×ª90¡ã
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)//µôÍ·
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)//¿ªÐ¡ÍÓÂÝÄ£Ê½
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)//ÔÆÌ¨²»¸úËæ
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)//Çå¿Õ¼üÅÌÊý¾Ý
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)//µõÉäË¿¸ËÎ¢µ÷£¨Õý£©
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)//µõÉäË¿¸ËÎ¢µ÷£¨¸º£©
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)//Ð¡ÍÓÂÝÄ£Ê½Ñ¡Ôñ
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)//UI³õÊ¼»¯
/* ----------------------- Data Struct ------------------------------------- */

extern RC_ctrl_t rc_ctrl;
void remote_control_init(void);
static void sbus_to_rc(uint8_t *sbus_buf);
//int16_t rc_filter_with_deadband(LowPassFilter *lpf, int16_t raw_value, uint8_t channel);

#endif
