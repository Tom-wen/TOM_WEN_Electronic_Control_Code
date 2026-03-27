#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include "main.h"
#include "bsp_usart.h"
#include "stdlib.h"

//#define DT7_rc_ctrl
//#define i6x_rc_ctrl
#define vtm_rc_ctrl



#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)
//拨杆三种状态
#define switch_up        ((uint16_t)1) //拨杆向上
#define switch_mid       ((uint16_t)3) //拨杆中间
#define switch_down        ((uint16_t)2) //拨杆向下

//左右拨杆
#define left_switch     rc_ctrl.rc.s[1]
#define right_switch    rc_ctrl.rc.s[0]

//侧拨杆
#define side_switch     rc_ctrl.rc.ch[4]
//侧拨杆状态
#define side_switch_on   (side_switch >= 600)    //侧拨杆开
#define side_switch_off  (side_switch <= -600)   //侧拨杆关

typedef struct __attribute__((packed))
{
        struct __attribute__((packed))
        {
                int16_t ch[5];
                char s[2];
        } rc;
        struct __attribute__((packed))
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

#define open_fire(s) (s == 1)
#define hold_fire(s) (s == 0)   
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)//疾跑
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)//UI初始化
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)//左转90°
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)//右转90°
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)//模式切换,底盘与云台分离模式，云台与底盘跟随模式
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)//开摩擦轮
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)//云台不跟随
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)//摆臂保护
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */

extern RC_ctrl_t rc_ctrl;
void remote_control_init(void);
static void sbus_to_rc(uint8_t *sbus_buf);




//i6x遥控器相关定义
#define I6X_FRAME_LENGTH 25u

#define I6X_SW_UP  ((int8_t)1)          
#define I6X_SW_MID   ((int8_t)0)        
#define I6X_SW_DOWN  ((int8_t)-1)        
#define i6x_switch_is_down(s)  (s == I6X_SW_DOWN)     
#define i6x_switch_is_mid(s)  (s == I6X_SW_MID)     
#define i6x_switch_is_up(s)  (s == I6X_SW_UP)

typedef struct {
        int16_t ch[6];
        int8_t s[4];
        uint8_t frame_lost;
        uint8_t failsafe;
} __attribute__((packed)) i6x_ctrl_t;
void sbus_to_i6x( uint8_t *sbus_data);
extern i6x_ctrl_t i6x_ctrl;


//VTM遥控器相关定义
/**
 * @brief VTM遥控器数据结构体
 */
typedef struct {
    // 摇杆通道（原始值）
    int16_t ch[4];
    
    // 摇杆映射（方便使用）
    int16_t left_x;   // 左摇杆X轴
    int16_t left_y;   // 左摇杆Y轴
    int16_t right_x;  // 右摇杆X轴
    int16_t right_y;  // 右摇杆Y轴
    
    // 开关和按钮
    uint8_t mode_sw:2; // 挡位切换开关: 0=C, 1=N, 2=S
    uint8_t pause:1;   // 暂停按键
    uint8_t fn_1:1;    // 自定义按键1（左）
    uint8_t fn_2:1;    // 自定义按键2（右）
    int16_t wheel:11; // 拨轮
    uint8_t trigger:1; // 扳机键
    
    // 鼠标数据
    int16_t mouse_x;   // 鼠标X轴移动速度
    int16_t mouse_y;   // 鼠标Y轴移动速度
    int16_t mouse_z;   // 鼠标滚轮速度
    
    // 鼠标按键（0=未按下，1=按下）
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    
    // 键盘数据（每个bit对应一个键）
    uint16_t key;
} vtm_rc_data_t;
void vtm_data_parse(uint8_t *vtm_buf);
extern vtm_rc_data_t vtm_rc_data;


#endif
