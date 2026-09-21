#ifndef __UI_TASK_H__
#define __UI_TASK_H__

#include "config.h"
#include "ui.h"
#include "Init.h"
#include "referee.h"

#ifdef COMPILE_UI

void dynamic_ui_speed_update();
void dynamic_ui_mode_update();
void dynamic_ui_aim_mode_update();
void dynamic_ui_pitch_update(INS_t *ins);
void dynamic_ui_power_update();
void dynamic_ui_gimbal_update(INS_t *ins);
void dynamic_ui_fire_update();

#endif

#endif
