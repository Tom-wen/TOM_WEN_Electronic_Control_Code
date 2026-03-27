#include "bsp_buzzer.h"

void buzzer_init() 
{ 
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

    htim12.Instance->ARR = 40000 - 1;       
    htim12.Instance->CCR2 = htim12.Instance->ARR / 2;
    HAL_Delay(250);  
    
    htim12.Instance->ARR = 28000 - 1;       
    htim12.Instance->CCR2 = htim12.Instance->ARR / 2;
    HAL_Delay(250);      

    htim12.Instance->ARR = 21000 - 1;       
    htim12.Instance->CCR2 = htim12.Instance->ARR / 2;
    HAL_Delay(300);  

    htim12.Instance->CCR2 = 0;
}

void buzzer_warning()
{
    while(1)
    {
        htim12.Instance->CCR2 =500;
        HAL_Delay(300);
        htim12.Instance->CCR2 =0;
        HAL_Delay(300);
    }
}

void buzzer_on()
{
    htim12.Instance->CCR2 =500;
}

void buzzer_off()
{
    htim12.Instance->CCR2 =0;
}









// ==================== 10MHz 计数时钟宏定义 ====================
// 基于 TIM12 计数时钟 = 10MHz (10,000,000 Hz)
// 计算公式: ARR = 10000000 / 频率 - 1

// 小星星使用的音符 (C5-A5)
#define NOTE_C5 19110  // 523.25 Hz  (1)
#define NOTE_D5 17026  // 587.33 Hz  (2)
#define NOTE_E5 15171  // 659.25 Hz  (3)
#define NOTE_F5 14320  // 698.46 Hz  (4)
#define NOTE_G5 12754  // 783.99 Hz  (5)
#define NOTE_A5 11363  // 880.00 Hz  (6)

// 休止符
#define NOTE_REST 0

// 节拍时长定义 (ms)
#define BEAT_4TH   400  // 四分音符 (一 闪)
#define BEAT_2ND   800  // 二分音符 (晶 -)

// ==================== 基础函数 ====================

/**
 * @brief 播放一个音符
 * @param arr_value  ARR值（音符宏定义）
 * @param duration_ms 时长（ms）
 */
void play_note(uint32_t arr_value, uint32_t duration_ms)
{
    if (arr_value == 0) {
        // 休止符
        buzzer_off();
        HAL_Delay(duration_ms);
        return;
    }
    
    // 设置频率 (ARR)
    htim12.Instance->ARR = arr_value;
    
    // 设置 50% 占空比 (音量适中)
    htim12.Instance->CCR2 = arr_value / 2;
    
    // 播放指定时长
    HAL_Delay(duration_ms);
    
    // 音符间短间隔 (使旋律更清晰)
    buzzer_off();
    HAL_Delay(30);
}

// ==================== 小星星 ====================

/**
 * @brief 播放《小星星》
 * @note 适配 10MHz 计数时钟
 */
void buzzer_play_twinkle_star(void)
{
    // 确保 PWM 已启动
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    
    // ========== 第一段 ==========
    // 1  1  5  5  |  6  6  5  -
    play_note(NOTE_C5, BEAT_4TH);  // 1
    play_note(NOTE_C5, BEAT_4TH);  // 1
    play_note(NOTE_G5, BEAT_4TH);  // 5
    play_note(NOTE_G5, BEAT_4TH);  // 5
    play_note(NOTE_A5, BEAT_4TH);  // 6
    play_note(NOTE_A5, BEAT_4TH);  // 6
    play_note(NOTE_G5, BEAT_2ND);  // 5 -
    
    // 4  4  3  3  |  2  2  1  -
    play_note(NOTE_F5, BEAT_4TH);  // 4
    play_note(NOTE_F5, BEAT_4TH);  // 4
    play_note(NOTE_E5, BEAT_4TH);  // 3
    play_note(NOTE_E5, BEAT_4TH);  // 3
    play_note(NOTE_D5, BEAT_4TH);  // 2
    play_note(NOTE_D5, BEAT_4TH);  // 2
    play_note(NOTE_C5, BEAT_2ND);  // 1 -
    
    // ========== 第二段 ==========
    // 5  5  4  4  |  3  3  2  -
    play_note(NOTE_G5, BEAT_4TH);  // 5
    play_note(NOTE_G5, BEAT_4TH);  // 5
    play_note(NOTE_F5, BEAT_4TH);  // 4
    play_note(NOTE_F5, BEAT_4TH);  // 4
    play_note(NOTE_E5, BEAT_4TH);  // 3
    play_note(NOTE_E5, BEAT_4TH);  // 3
    play_note(NOTE_D5, BEAT_2ND);  // 2 -
    
    // 5  5  4  4  |  3  3  2  -
    play_note(NOTE_G5, BEAT_4TH);  // 5
    play_note(NOTE_G5, BEAT_4TH);  // 5
    play_note(NOTE_F5, BEAT_4TH);  // 4
    play_note(NOTE_F5, BEAT_4TH);  // 4
    play_note(NOTE_E5, BEAT_4TH);  // 3
    play_note(NOTE_E5, BEAT_4TH);  // 3
    play_note(NOTE_D5, BEAT_2ND);  // 2 -
    
    // ========== 第三段 (重复第一段) ==========
    // 1  1  5  5  |  6  6  5  -
    play_note(NOTE_C5, BEAT_4TH);  // 1
    play_note(NOTE_C5, BEAT_4TH);  // 1
    play_note(NOTE_G5, BEAT_4TH);  // 5
    play_note(NOTE_G5, BEAT_4TH);  // 5
    play_note(NOTE_A5, BEAT_4TH);  // 6
    play_note(NOTE_A5, BEAT_4TH);  // 6
    play_note(NOTE_G5, BEAT_2ND);  // 5 -
    
    // 4  4  3  3  |  2  2  1  -
    play_note(NOTE_F5, BEAT_4TH);  // 4
    play_note(NOTE_F5, BEAT_4TH);  // 4
    play_note(NOTE_E5, BEAT_4TH);  // 3
    play_note(NOTE_E5, BEAT_4TH);  // 3
    play_note(NOTE_D5, BEAT_4TH);  // 2
    play_note(NOTE_D5, BEAT_4TH);  // 2
    play_note(NOTE_C5, BEAT_2ND);  // 1 -
    
    // 最后关闭蜂鸣器
    buzzer_off();
}