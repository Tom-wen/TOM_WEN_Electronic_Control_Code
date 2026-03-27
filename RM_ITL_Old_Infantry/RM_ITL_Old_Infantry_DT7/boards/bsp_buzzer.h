#ifndef __BSP_BUZZER_H__
#define __BSP_BUZZER_H__

#include "stm32h7xx.h"
#include "tim.h"

void buzzer_init();
void buzzer_warning();
void buzzer_on();
void buzzer_off();
void play_note(uint32_t arr_value, uint32_t duration_ms);
void buzzer_play_twinkle_star(void);

#endif
