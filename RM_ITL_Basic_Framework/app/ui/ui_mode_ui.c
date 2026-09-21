//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"


ui_string_frame_t ui_mode_ui_group1_0;
ui_interface_string_t* ui_mode_ui_group1_follow_mode = &(ui_mode_ui_group1_0.option);

void _ui_init_mode_ui_group1_0() {
    ui_mode_ui_group1_0.option.figure_name[0] = 1;
    ui_mode_ui_group1_0.option.figure_name[1] = 0;
    ui_mode_ui_group1_0.option.figure_name[2] = 0;
    ui_mode_ui_group1_0.option.operate_type = 1;

    ui_mode_ui_group1_follow_mode->figure_type = 7;
    ui_mode_ui_group1_follow_mode->operate_type = 1;
    ui_mode_ui_group1_follow_mode->layer = 0;
    ui_mode_ui_group1_follow_mode->color = 8;
    ui_mode_ui_group1_follow_mode->start_x = 754;
    ui_mode_ui_group1_follow_mode->start_y = 187;
    ui_mode_ui_group1_follow_mode->width = 2;
    ui_mode_ui_group1_follow_mode->font_size = 20;
    ui_mode_ui_group1_follow_mode->str_length = 21;
    strcpy(ui_mode_ui_group1_follow_mode->string, "CHASSIS_FOLLOW_GIMBAL");


    ui_proc_string_frame(&ui_mode_ui_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_mode_ui_group1_0, sizeof(ui_mode_ui_group1_0));
}

void _ui_update_mode_ui_group1_0() {
    ui_mode_ui_group1_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_mode_ui_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_mode_ui_group1_0, sizeof(ui_mode_ui_group1_0));
}

void _ui_remove_mode_ui_group1_0() {
    ui_mode_ui_group1_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_mode_ui_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_mode_ui_group1_0, sizeof(ui_mode_ui_group1_0));
}
ui_string_frame_t ui_mode_ui_group1_1;
ui_interface_string_t* ui_mode_ui_group1_rotate_mode = &(ui_mode_ui_group1_1.option);

void _ui_init_mode_ui_group1_1() {
    ui_mode_ui_group1_1.option.figure_name[0] = 1;
    ui_mode_ui_group1_1.option.figure_name[1] = 0;
    ui_mode_ui_group1_1.option.figure_name[2] = 1;
    ui_mode_ui_group1_1.option.operate_type = 1;

    ui_mode_ui_group1_rotate_mode->figure_type = 7;
    ui_mode_ui_group1_rotate_mode->operate_type = 1;
    ui_mode_ui_group1_rotate_mode->layer = 0;
    ui_mode_ui_group1_rotate_mode->color = 8;
    ui_mode_ui_group1_rotate_mode->start_x = 811;
    ui_mode_ui_group1_rotate_mode->start_y = 138;
    ui_mode_ui_group1_rotate_mode->width = 2;
    ui_mode_ui_group1_rotate_mode->font_size = 20;
    ui_mode_ui_group1_rotate_mode->str_length = 14;
    strcpy(ui_mode_ui_group1_rotate_mode->string, "CHASSIS_ROTATE");


    ui_proc_string_frame(&ui_mode_ui_group1_1);
    SEND_MESSAGE((uint8_t *) &ui_mode_ui_group1_1, sizeof(ui_mode_ui_group1_1));
}

void _ui_update_mode_ui_group1_1() {
    ui_mode_ui_group1_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_mode_ui_group1_1);
    SEND_MESSAGE((uint8_t *) &ui_mode_ui_group1_1, sizeof(ui_mode_ui_group1_1));
}

void _ui_remove_mode_ui_group1_1() {
    ui_mode_ui_group1_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_mode_ui_group1_1);
    SEND_MESSAGE((uint8_t *) &ui_mode_ui_group1_1, sizeof(ui_mode_ui_group1_1));
}
ui_string_frame_t ui_mode_ui_group1_2;
ui_interface_string_t* ui_mode_ui_group1_auto_arm = &(ui_mode_ui_group1_2.option);

void _ui_init_mode_ui_group1_2() {
    ui_mode_ui_group1_2.option.figure_name[0] = 1;
    ui_mode_ui_group1_2.option.figure_name[1] = 0;
    ui_mode_ui_group1_2.option.figure_name[2] = 2;
    ui_mode_ui_group1_2.option.operate_type = 1;

    ui_mode_ui_group1_auto_arm->figure_type = 7;
    ui_mode_ui_group1_auto_arm->operate_type = 1;
    ui_mode_ui_group1_auto_arm->layer = 0;
    ui_mode_ui_group1_auto_arm->color = 8;
    ui_mode_ui_group1_auto_arm->start_x = 860;
    ui_mode_ui_group1_auto_arm->start_y = 243;
    ui_mode_ui_group1_auto_arm->width = 2;
    ui_mode_ui_group1_auto_arm->font_size = 20;
    ui_mode_ui_group1_auto_arm->str_length = 8;
    strcpy(ui_mode_ui_group1_auto_arm->string, "AUTO_ARM");


    ui_proc_string_frame(&ui_mode_ui_group1_2);
    SEND_MESSAGE((uint8_t *) &ui_mode_ui_group1_2, sizeof(ui_mode_ui_group1_2));
}

void _ui_update_mode_ui_group1_2() {
    ui_mode_ui_group1_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_mode_ui_group1_2);
    SEND_MESSAGE((uint8_t *) &ui_mode_ui_group1_2, sizeof(ui_mode_ui_group1_2));
}

void _ui_remove_mode_ui_group1_2() {
    ui_mode_ui_group1_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_mode_ui_group1_2);
    SEND_MESSAGE((uint8_t *) &ui_mode_ui_group1_2, sizeof(ui_mode_ui_group1_2));
}

void ui_init_mode_ui_group1() {
    _ui_init_mode_ui_group1_0();
    _ui_init_mode_ui_group1_1();
    _ui_init_mode_ui_group1_2();
}

void ui_update_mode_ui_group1() {
    _ui_update_mode_ui_group1_0();
    _ui_update_mode_ui_group1_1();
    _ui_update_mode_ui_group1_2();
}

void ui_remove_mode_ui_group1() {
    _ui_remove_mode_ui_group1_0();
    _ui_remove_mode_ui_group1_1();
    _ui_remove_mode_ui_group1_2();
}

ui_1_frame_t ui_mode_ui_group2_0;

ui_interface_line_t *ui_mode_ui_group2_speed_number = (ui_interface_line_t*)&(ui_mode_ui_group2_0.data[0]);

void _ui_init_mode_ui_group2_0() {
    for (int i = 0; i < 1; i++) {
        ui_mode_ui_group2_0.data[i].figure_name[0] = 1;
        ui_mode_ui_group2_0.data[i].figure_name[1] = 1;
        ui_mode_ui_group2_0.data[i].figure_name[2] = i + 0;
        ui_mode_ui_group2_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_mode_ui_group2_0.data[i].operate_type = 0;
    }

    ui_mode_ui_group2_speed_number->figure_type = 0;
    ui_mode_ui_group2_speed_number->operate_type = 1;
    ui_mode_ui_group2_speed_number->layer = 0;
    ui_mode_ui_group2_speed_number->color = 0;
    ui_mode_ui_group2_speed_number->start_x = 1416;
    ui_mode_ui_group2_speed_number->start_y = 600;
    ui_mode_ui_group2_speed_number->width = 28;
    ui_mode_ui_group2_speed_number->end_x = 1416;
    ui_mode_ui_group2_speed_number->end_y = 660;


    ui_proc_1_frame(&ui_mode_ui_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_mode_ui_group2_0, sizeof(ui_mode_ui_group2_0));
}

void _ui_update_mode_ui_group2_0() {
    for (int i = 0; i < 1; i++) {
        ui_mode_ui_group2_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_mode_ui_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_mode_ui_group2_0, sizeof(ui_mode_ui_group2_0));
}

void _ui_remove_mode_ui_group2_0() {
    for (int i = 0; i < 1; i++) {
        ui_mode_ui_group2_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_mode_ui_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_mode_ui_group2_0, sizeof(ui_mode_ui_group2_0));
}


void ui_init_mode_ui_group2() {
    _ui_init_mode_ui_group2_0();
}

void ui_update_mode_ui_group2() {
    _ui_update_mode_ui_group2_0();
}

void ui_remove_mode_ui_group2() {
    _ui_remove_mode_ui_group2_0();
}

