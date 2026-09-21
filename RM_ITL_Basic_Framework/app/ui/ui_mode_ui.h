//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_mode_ui_H
#define UI_mode_ui_H

#include "ui_interface.h"

extern ui_interface_string_t *ui_mode_ui_group1_follow_mode;
extern ui_interface_string_t *ui_mode_ui_group1_rotate_mode;
extern ui_interface_string_t *ui_mode_ui_group1_auto_arm;

void ui_init_mode_ui_group1();
void ui_update_mode_ui_group1();
void ui_remove_mode_ui_group1();

extern ui_interface_line_t *ui_mode_ui_group2_speed_number;

void ui_init_mode_ui_group2();
void ui_update_mode_ui_group2();
void ui_remove_mode_ui_group2();


#endif // UI_mode_ui_H
