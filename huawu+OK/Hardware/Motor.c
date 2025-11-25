#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "MOTOR.h"

/**
  * 鍑�    鏁帮細鐩存祦鐢垫満鍒濆鍖�
  * 鍙�    鏁帮細鏃�
  * 杩� 鍥� 鍊硷細鏃�
  */
void Motor_Init(void)
{
    /*寮€鍚椂閽�*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 鍙厤缃富鐢垫満鎺у埗寮曡剼
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = MOTOR_LEFT_FORWARD | MOTOR_LEFT_BACKWARD | 
                                 MOTOR_RIGHT_FORWARD | MOTOR_RIGHT_BACKWARD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 濡傛灉闇€瑕侀澶栫殑娴嬭瘯鐢垫満锛屽崟鐙厤缃�
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_7;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    PWM_Init();
}

//void Motor_B_Init(void)
//{
//	/*寮€鍚椂閽�*/
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//寮€鍚疓PIOA鐨勬椂閽�
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12; //|GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);						//灏哖A4鍜孭A5寮曡剼鍒濆鍖栦负鎺ㄦ尳杈撳嚭	
//	PWM_Init();													//鍒濆鍖栫洿娴佺數鏈虹殑搴曞眰PWM
//}

/**
  * 鍑�    鏁帮細鐩存祦鐢垫満璁剧疆閫熷害
  * 鍙�    鏁帮細Speed 瑕佽缃殑閫熷害锛岃寖鍥达細-100~100
  * 杩� 鍥� 鍊硷細鏃�
  */
void Motor_SetSpeed(int8_t Speed)
{
    uint16_t pwm_value;
    
    // 灏哠peed(-100鍒�100)鏄犲皠鍒癙WM(0鍒�999)
    if (Speed >= 0)
    {
        // 姝ｈ浆
        GPIO_SetBits(GPIOC, GPIO_Pin_0);    // 宸︾數鏈烘杞�
        GPIO_ResetBits(GPIOC, GPIO_Pin_1);  // 宸︾數鏈哄弽杞�
        GPIO_SetBits(GPIOC, GPIO_Pin_2);    // 鍙崇數鏈烘杞�
        GPIO_ResetBits(GPIOC, GPIO_Pin_3);  // 鍙崇數鏈哄弽杞�
//---------------------------------test--------------------------------//
        GPIO_SetBits(GPIOC, GPIO_Pin_4);    // 宸︾數鏈烘杞�
        GPIO_ResetBits(GPIOC, GPIO_Pin_5);  // 宸︾數鏈哄弽杞�
        GPIO_SetBits(GPIOD, GPIO_Pin_3);    // 鍙崇數鏈烘杞�
        GPIO_ResetBits(GPIOD, GPIO_Pin_7);  // 鍙崇數鏈哄弽杞�
      
        pwm_value = (uint16_t)((999 * Speed) / 100);  // 鏄犲皠鍒�0-999
    }
    else
    {
        // 鍙嶈浆
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);  // 宸︾數鏈烘杞�
        GPIO_SetBits(GPIOC, GPIO_Pin_1);    // 宸︾數鏈哄弽杞�
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);  // 鍙崇數鏈烘杞�
        GPIO_SetBits(GPIOC, GPIO_Pin_3);    // 鍙崇數鏈哄弽杞�
//----------------------------------------TEST-------------------------------------//
        GPIO_ResetBits(GPIOC, GPIO_Pin_4);  // 宸︾數鏈烘杞�
        GPIO_SetBits(GPIOC, GPIO_Pin_5);    // 宸︾數鏈哄弽杞�
        GPIO_ResetBits(GPIOD, GPIO_Pin_3);  // 鍙崇數鏈烘杞�
        GPIO_SetBits(GPIOD, GPIO_Pin_7);    // 鍙崇數鏈哄弽杞�
			
        pwm_value = (uint16_t)((999 * (-Speed)) / 100);  // 鏄犲皠鍒�0-999
    }
    
    // 璁剧疆涓や釜鐢垫満鐨凱WM
    PWM_SetCompare3(pwm_value);  // 宸︾數鏈� - PB0
    PWM_SetCompare4(pwm_value);  // 鍙崇數鏈� - PB1
		test_motor_left(pwm_value);
		test_motor_right(pwm_value);
}
void set_motor_speeds(float left_speed, float right_speed)
{
    uint16_t left_pwm, right_pwm;
    
    // 澶勭悊宸︾數鏈�
    if (left_speed >= 0)
    {
        // 宸︾數鏈烘杞�
        GPIO_SetBits(GPIOC, GPIO_Pin_0);    // 宸︾數鏈烘杞�
        GPIO_ResetBits(GPIOC, GPIO_Pin_1);  // 宸︾數鏈哄弽杞�
        left_pwm = (uint16_t)((999 * left_speed) / 100);  // 鏄犲皠鍒�0-999
    }
    else
    {
        // 宸︾數鏈哄弽杞�
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);  // 宸︾數鏈烘杞�
        GPIO_SetBits(GPIOC, GPIO_Pin_1);    // 宸︾數鏈哄弽杞�
        left_pwm = (uint16_t)((999 * (-left_speed)) / 100);  // 鏄犲皠鍒�0-999
    }
    
    // 澶勭悊鍙崇數鏈�
    if (right_speed >= 0)
    {
        // 鍙崇數鏈烘杞�
        GPIO_SetBits(GPIOC, GPIO_Pin_2);    // 鍙崇數鏈烘杞�
        GPIO_ResetBits(GPIOC, GPIO_Pin_3);  // 鍙崇數鏈哄弽杞�
        right_pwm = (uint16_t)((999 * right_speed) / 100);  // 鏄犲皠鍒�0-999
    }
    else
    {
        // 鍙崇數鏈哄弽杞�
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);  // 鍙崇數鏈烘杞�
        GPIO_SetBits(GPIOC, GPIO_Pin_3);    // 鍙崇數鏈哄弽杞�
        right_pwm = (uint16_t)((999 * (-right_speed)) / 100);  // 鏄犲皠鍒�0-999
    }
    
    // 璁剧疆涓や釜鐢垫満鐨凱WM
    PWM_SetCompare3(left_pwm);   // 宸︾數鏈�
    PWM_SetCompare4(right_pwm);  // 鍙崇數鏈�
}




//楹﹁疆瑙ｇ畻鍏紡 鐢变簬涓嶉渶瑕佽浆鍚憌z=0
//    vt_lf = chassis_vx - chassis_vy - chassis_cmd_recv.wz * MOTOR_DISTANCE_TO_CENTER;
//    vt_rf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * MOTOR_DISTANCE_TO_CENTER;
//    vt_rb = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * MOTOR_DISTANCE_TO_CENTER;
//    vt_lb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * MOTOR_DISTANCE_TO_CENTER;

void Motor_MECNAMU_SetSpeed(float chassis_vx,float chassis_vy)
{
	  uint16_t vt_lf_pwm;
	  uint16_t vt_rf_pwm;
	  uint16_t vt_lb_pwm;
	  uint16_t vt_rb_pwm;
	
	  float vt_lf;
	  float vt_rf;
	  float vt_lb;
	  float vt_rb;
		vt_lf = -(-chassis_vx + chassis_vy); //(chassis_vx - chassis_vy);//鍙冲悗杞�//(chassis_vx - chassis_vy)
    vt_rf = -(chassis_vx + chassis_vy); ;//(-chassis_vx - chassis_vy);//宸﹀悗杞�//(-chassis_vx - chassis_vy)
    vt_rb = (chassis_vx - chassis_vy); ;//(-chassis_vx + chassis_vy);//宸﹀墠杞�//(-chassis_vx + chassis_vy);
    vt_lb = (-chassis_vx - chassis_vy); //(chassis_vx + chassis_vy);//鍙冲墠杞�//(chassis_vx + chassis_vy)
    
    // 澶勭悊宸︾數鏈�
    if (vt_lf >= 0)
    {
        // 宸︾數鏈烘杞�
        GPIO_SetBits(GPIOC, GPIO_Pin_0);    // 宸︾數鏈烘杞�
        GPIO_ResetBits(GPIOC, GPIO_Pin_1);  // 宸︾數鏈哄弽杞�
        vt_lf_pwm = (uint16_t)((999 * vt_lf) / 100);  // 鏄犲皠鍒�0-999
    }
    else
    {
        // 宸︾數鏈哄弽杞�
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);  // 宸︾數鏈烘杞�
        GPIO_SetBits(GPIOC, GPIO_Pin_1);    // 宸︾數鏈哄弽杞�
        vt_lf_pwm = (uint16_t)((999 * (-vt_lf)) / 100);  // 鏄犲皠鍒�0-999
    }
    
    // 澶勭悊鍙崇數鏈�
    if (vt_rf >= 0)
    {
        // 鍙崇數鏈烘杞�
        GPIO_SetBits(GPIOC, GPIO_Pin_2);    // 鍙崇數鏈烘杞�
        GPIO_ResetBits(GPIOC, GPIO_Pin_3);  // 鍙崇數鏈哄弽杞�
        vt_rf_pwm = (uint16_t)((999 * vt_rf) / 100);  // 鏄犲皠鍒�0-999
    }
    else
    {
        // 鍙崇數鏈哄弽杞�
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);  // 鍙崇數鏈烘杞�
        GPIO_SetBits(GPIOC, GPIO_Pin_3);    // 鍙崇數鏈哄弽杞�
        vt_rf_pwm = (uint16_t)((999 * (-vt_rf)) / 100);  // 鏄犲皠鍒�0-999
    }
		    if (vt_rb >= 0)
    {
        // 宸︾數鏈烘杞�
        GPIO_SetBits(GPIOC, GPIO_Pin_4);    // 宸︾數鏈烘杞�
        GPIO_ResetBits(GPIOC, GPIO_Pin_5);  // 宸︾數鏈哄弽杞�
        vt_rb_pwm = (uint16_t)((999 * vt_rb) / 100);  // 鏄犲皠鍒�0-999
    }
    else
    {
        // 宸︾數鏈哄弽杞�
        GPIO_ResetBits(GPIOC, GPIO_Pin_4);  // 宸︾數鏈烘杞�
        GPIO_SetBits(GPIOC, GPIO_Pin_5);    // 宸︾數鏈哄弽杞�
        vt_rb_pwm = (uint16_t)((999 * (-vt_rb)) / 100);  // 鏄犲皠鍒�0-999
    }
    
    // 澶勭悊鍙崇數鏈�
    if (vt_lb >= 0)
    {
        // 鍙崇數鏈烘杞�
        GPIO_SetBits(GPIOD, GPIO_Pin_3);    // 鍙崇數鏈烘杞�
        GPIO_ResetBits(GPIOD, GPIO_Pin_7);  // 鍙崇數鏈哄弽杞�
        vt_lb_pwm = (uint16_t)((999 * vt_lb) / 100);  // 鏄犲皠鍒�0-999
    }
    else
    {
        // 鍙崇數鏈哄弽杞�
        GPIO_ResetBits(GPIOD, GPIO_Pin_3);  // 鍙崇數鏈烘杞�
        GPIO_SetBits(GPIOD, GPIO_Pin_7);    // 鍙崇數鏈哄弽杞�
        vt_lb_pwm = (uint16_t)((999 * (-vt_lb)) / 100);  // 鏄犲皠鍒�0-999
    }
		
		


    
    // 璁剧疆涓や釜鐢垫満鐨凱WM
    PWM_SetCompare3(vt_lf_pwm);  // 宸︾數鏈� - PB0
    PWM_SetCompare4(vt_rf_pwm);  // 鍙崇數鏈� - PB1
	test_motor_left(1.19*vt_rb_pwm);//1.19*
	test_motor_right(vt_lb_pwm);
}

