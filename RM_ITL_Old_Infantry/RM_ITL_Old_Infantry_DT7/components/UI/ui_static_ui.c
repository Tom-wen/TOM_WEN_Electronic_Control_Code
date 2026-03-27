//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_7_frame_t ui_static_ui_Ungroup_0;

ui_interface_line_t *ui_static_ui_Ungroup_mid_line = (ui_interface_line_t*)&(ui_static_ui_Ungroup_0.data[0]);
ui_interface_line_t *ui_static_ui_Ungroup_vertical_Line = (ui_interface_line_t*)&(ui_static_ui_Ungroup_0.data[1]);
ui_interface_arc_t *ui_static_ui_Ungroup_downArc = (ui_interface_arc_t*)&(ui_static_ui_Ungroup_0.data[2]);
ui_interface_line_t *ui_static_ui_Ungroup_mode_line1 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_0.data[3]);
ui_interface_line_t *ui_static_ui_Ungroup_mode_line2 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_0.data[4]);
ui_interface_rect_t *ui_static_ui_Ungroup_speed_mode1 = (ui_interface_rect_t*)&(ui_static_ui_Ungroup_0.data[5]);
ui_interface_line_t *ui_static_ui_Ungroup_mid_line1 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_0.data[6]);

void _ui_init_static_ui_Ungroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_0.data[i].figure_name[0] = 3;
        ui_static_ui_Ungroup_0.data[i].figure_name[1] = 0;
        ui_static_ui_Ungroup_0.data[i].figure_name[2] = i + 0;
        ui_static_ui_Ungroup_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_static_ui_Ungroup_0.data[i].operate_type = 0;
    }

    ui_static_ui_Ungroup_mid_line->figure_type = 0;
    ui_static_ui_Ungroup_mid_line->operate_type = 1;
    ui_static_ui_Ungroup_mid_line->layer = 0;
    ui_static_ui_Ungroup_mid_line->color = 8;
    ui_static_ui_Ungroup_mid_line->start_x = 760;
    ui_static_ui_Ungroup_mid_line->start_y = 540;
    ui_static_ui_Ungroup_mid_line->width = 1;
    ui_static_ui_Ungroup_mid_line->end_x = 1160;
    ui_static_ui_Ungroup_mid_line->end_y = 540;

    ui_static_ui_Ungroup_vertical_Line->figure_type = 0;
    ui_static_ui_Ungroup_vertical_Line->operate_type = 1;
    ui_static_ui_Ungroup_vertical_Line->layer = 0;
    ui_static_ui_Ungroup_vertical_Line->color = 8;
    ui_static_ui_Ungroup_vertical_Line->start_x = 960;
    ui_static_ui_Ungroup_vertical_Line->start_y = 641;
    ui_static_ui_Ungroup_vertical_Line->width = 1;
    ui_static_ui_Ungroup_vertical_Line->end_x = 960;
    ui_static_ui_Ungroup_vertical_Line->end_y = 300;

    ui_static_ui_Ungroup_downArc->figure_type = 4;
    ui_static_ui_Ungroup_downArc->operate_type = 1;
    ui_static_ui_Ungroup_downArc->layer = 0;
    ui_static_ui_Ungroup_downArc->color = 8;
    ui_static_ui_Ungroup_downArc->start_x = 960;
    ui_static_ui_Ungroup_downArc->start_y = 546;
    ui_static_ui_Ungroup_downArc->width = 1;
    ui_static_ui_Ungroup_downArc->start_angle = 338;
    ui_static_ui_Ungroup_downArc->end_angle = 22;
    ui_static_ui_Ungroup_downArc->rx = 300;
    ui_static_ui_Ungroup_downArc->ry = 300;

    ui_static_ui_Ungroup_mode_line1->figure_type = 0;
    ui_static_ui_Ungroup_mode_line1->operate_type = 1;
    ui_static_ui_Ungroup_mode_line1->layer = 0;
    ui_static_ui_Ungroup_mode_line1->color = 8;
    ui_static_ui_Ungroup_mode_line1->start_x = 857;
    ui_static_ui_Ungroup_mode_line1->start_y = 294;
    ui_static_ui_Ungroup_mode_line1->width = 1;
    ui_static_ui_Ungroup_mode_line1->end_x = 849;
    ui_static_ui_Ungroup_mode_line1->end_y = 265;

    ui_static_ui_Ungroup_mode_line2->figure_type = 0;
    ui_static_ui_Ungroup_mode_line2->operate_type = 1;
    ui_static_ui_Ungroup_mode_line2->layer = 0;
    ui_static_ui_Ungroup_mode_line2->color = 8;
    ui_static_ui_Ungroup_mode_line2->start_x = 1063;
    ui_static_ui_Ungroup_mode_line2->start_y = 294;
    ui_static_ui_Ungroup_mode_line2->width = 1;
    ui_static_ui_Ungroup_mode_line2->end_x = 1071;
    ui_static_ui_Ungroup_mode_line2->end_y = 265;

    ui_static_ui_Ungroup_speed_mode1->figure_type = 1;
    ui_static_ui_Ungroup_speed_mode1->operate_type = 1;
    ui_static_ui_Ungroup_speed_mode1->layer = 0;
    ui_static_ui_Ungroup_speed_mode1->color = 8;
    ui_static_ui_Ungroup_speed_mode1->start_x = 1400;
    ui_static_ui_Ungroup_speed_mode1->start_y = 620;
    ui_static_ui_Ungroup_speed_mode1->width = 1;
    ui_static_ui_Ungroup_speed_mode1->end_x = 1430;
    ui_static_ui_Ungroup_speed_mode1->end_y = 650;

    ui_static_ui_Ungroup_mid_line1->figure_type = 0;
    ui_static_ui_Ungroup_mid_line1->operate_type = 1;
    ui_static_ui_Ungroup_mid_line1->layer = 0;
    ui_static_ui_Ungroup_mid_line1->color = 8;
    ui_static_ui_Ungroup_mid_line1->start_x = 920;
    ui_static_ui_Ungroup_mid_line1->start_y = 520;
    ui_static_ui_Ungroup_mid_line1->width = 1;
    ui_static_ui_Ungroup_mid_line1->end_x = 1000;
    ui_static_ui_Ungroup_mid_line1->end_y = 520;


    ui_proc_7_frame(&ui_static_ui_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_0, sizeof(ui_static_ui_Ungroup_0));
}

void _ui_update_static_ui_Ungroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_static_ui_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_0, sizeof(ui_static_ui_Ungroup_0));
}

void _ui_remove_static_ui_Ungroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_static_ui_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_0, sizeof(ui_static_ui_Ungroup_0));
}
ui_7_frame_t ui_static_ui_Ungroup_1;

ui_interface_arc_t *ui_static_ui_Ungroup_energy_arc1 = (ui_interface_arc_t*)&(ui_static_ui_Ungroup_1.data[0]);
ui_interface_arc_t *ui_static_ui_Ungroup_energy_arc2 = (ui_interface_arc_t*)&(ui_static_ui_Ungroup_1.data[1]);
ui_interface_line_t *ui_static_ui_Ungroup_energy_line1 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_1.data[2]);
ui_interface_line_t *ui_static_ui_Ungroup_energy_line2 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_1.data[3]);
ui_interface_line_t *ui_static_ui_Ungroup_mid_line2 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_1.data[4]);
ui_interface_line_t *ui_static_ui_Ungroup_mid_line3 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_1.data[5]);
ui_interface_line_t *ui_static_ui_Ungroup_mid_line4 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_1.data[6]);

void _ui_init_static_ui_Ungroup_1() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_1.data[i].figure_name[0] = 3;
        ui_static_ui_Ungroup_1.data[i].figure_name[1] = 0;
        ui_static_ui_Ungroup_1.data[i].figure_name[2] = i + 7;
        ui_static_ui_Ungroup_1.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_static_ui_Ungroup_1.data[i].operate_type = 0;
    }

    ui_static_ui_Ungroup_energy_arc1->figure_type = 4;
    ui_static_ui_Ungroup_energy_arc1->operate_type = 1;
    ui_static_ui_Ungroup_energy_arc1->layer = 0;
    ui_static_ui_Ungroup_energy_arc1->color = 8;
    ui_static_ui_Ungroup_energy_arc1->start_x = 950;
    ui_static_ui_Ungroup_energy_arc1->start_y = 540;
    ui_static_ui_Ungroup_energy_arc1->width = 1;
    ui_static_ui_Ungroup_energy_arc1->start_angle = 230;
    ui_static_ui_Ungroup_energy_arc1->end_angle = 270;
    ui_static_ui_Ungroup_energy_arc1->rx = 370;
    ui_static_ui_Ungroup_energy_arc1->ry = 370;

    ui_static_ui_Ungroup_energy_arc2->figure_type = 4;
    ui_static_ui_Ungroup_energy_arc2->operate_type = 1;
    ui_static_ui_Ungroup_energy_arc2->layer = 0;
    ui_static_ui_Ungroup_energy_arc2->color = 8;
    ui_static_ui_Ungroup_energy_arc2->start_x = 975;
    ui_static_ui_Ungroup_energy_arc2->start_y = 540;
    ui_static_ui_Ungroup_energy_arc2->width = 1;
    ui_static_ui_Ungroup_energy_arc2->start_angle = 233;
    ui_static_ui_Ungroup_energy_arc2->end_angle = 269;
    ui_static_ui_Ungroup_energy_arc2->rx = 369;
    ui_static_ui_Ungroup_energy_arc2->ry = 369;

    ui_static_ui_Ungroup_energy_line1->figure_type = 0;
    ui_static_ui_Ungroup_energy_line1->operate_type = 1;
    ui_static_ui_Ungroup_energy_line1->layer = 0;
    ui_static_ui_Ungroup_energy_line1->color = 8;
    ui_static_ui_Ungroup_energy_line1->start_x = 666;
    ui_static_ui_Ungroup_energy_line1->start_y = 775;
    ui_static_ui_Ungroup_energy_line1->width = 1;
    ui_static_ui_Ungroup_energy_line1->end_x = 680;
    ui_static_ui_Ungroup_energy_line1->end_y = 759;

    ui_static_ui_Ungroup_energy_line2->figure_type = 0;
    ui_static_ui_Ungroup_energy_line2->operate_type = 1;
    ui_static_ui_Ungroup_energy_line2->layer = 0;
    ui_static_ui_Ungroup_energy_line2->color = 8;
    ui_static_ui_Ungroup_energy_line2->start_x = 579;
    ui_static_ui_Ungroup_energy_line2->start_y = 539;
    ui_static_ui_Ungroup_energy_line2->width = 1;
    ui_static_ui_Ungroup_energy_line2->end_x = 605;
    ui_static_ui_Ungroup_energy_line2->end_y = 547;

    ui_static_ui_Ungroup_mid_line2->figure_type = 0;
    ui_static_ui_Ungroup_mid_line2->operate_type = 1;
    ui_static_ui_Ungroup_mid_line2->layer = 0;
    ui_static_ui_Ungroup_mid_line2->color = 8;
    ui_static_ui_Ungroup_mid_line2->start_x = 900;
    ui_static_ui_Ungroup_mid_line2->start_y = 500;
    ui_static_ui_Ungroup_mid_line2->width = 1;
    ui_static_ui_Ungroup_mid_line2->end_x = 1020;
    ui_static_ui_Ungroup_mid_line2->end_y = 500;

    ui_static_ui_Ungroup_mid_line3->figure_type = 0;
    ui_static_ui_Ungroup_mid_line3->operate_type = 1;
    ui_static_ui_Ungroup_mid_line3->layer = 0;
    ui_static_ui_Ungroup_mid_line3->color = 8;
    ui_static_ui_Ungroup_mid_line3->start_x = 880;
    ui_static_ui_Ungroup_mid_line3->start_y = 480;
    ui_static_ui_Ungroup_mid_line3->width = 1;
    ui_static_ui_Ungroup_mid_line3->end_x = 1041;
    ui_static_ui_Ungroup_mid_line3->end_y = 480;

    ui_static_ui_Ungroup_mid_line4->figure_type = 0;
    ui_static_ui_Ungroup_mid_line4->operate_type = 1;
    ui_static_ui_Ungroup_mid_line4->layer = 0;
    ui_static_ui_Ungroup_mid_line4->color = 8;
    ui_static_ui_Ungroup_mid_line4->start_x = 860;
    ui_static_ui_Ungroup_mid_line4->start_y = 460;
    ui_static_ui_Ungroup_mid_line4->width = 1;
    ui_static_ui_Ungroup_mid_line4->end_x = 1060;
    ui_static_ui_Ungroup_mid_line4->end_y = 460;


    ui_proc_7_frame(&ui_static_ui_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_1, sizeof(ui_static_ui_Ungroup_1));
}

void _ui_update_static_ui_Ungroup_1() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_1.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_static_ui_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_1, sizeof(ui_static_ui_Ungroup_1));
}

void _ui_remove_static_ui_Ungroup_1() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_1.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_static_ui_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_1, sizeof(ui_static_ui_Ungroup_1));
}
ui_5_frame_t ui_static_ui_Ungroup_2;

ui_interface_ellipse_t *ui_static_ui_Ungroup_mid_Ellipse = (ui_interface_ellipse_t*)&(ui_static_ui_Ungroup_2.data[0]);
ui_interface_line_t *ui_static_ui_Ungroup_left_line = (ui_interface_line_t*)&(ui_static_ui_Ungroup_2.data[1]);
ui_interface_line_t *ui_static_ui_Ungroup_right_line = (ui_interface_line_t*)&(ui_static_ui_Ungroup_2.data[2]);
ui_interface_arc_t *ui_static_ui_Ungroup_upArc = (ui_interface_arc_t*)&(ui_static_ui_Ungroup_2.data[3]);

void _ui_init_static_ui_Ungroup_2() {
    for (int i = 0; i < 4; i++) {
        ui_static_ui_Ungroup_2.data[i].figure_name[0] = 3;
        ui_static_ui_Ungroup_2.data[i].figure_name[1] = 0;
        ui_static_ui_Ungroup_2.data[i].figure_name[2] = i + 14;
        ui_static_ui_Ungroup_2.data[i].operate_type = 1;
    }
    for (int i = 4; i < 5; i++) {
        ui_static_ui_Ungroup_2.data[i].operate_type = 0;
    }

    ui_static_ui_Ungroup_mid_Ellipse->figure_type = 3;
    ui_static_ui_Ungroup_mid_Ellipse->operate_type = 1;
    ui_static_ui_Ungroup_mid_Ellipse->layer = 0;
    ui_static_ui_Ungroup_mid_Ellipse->color = 8;
    ui_static_ui_Ungroup_mid_Ellipse->start_x = 960;
    ui_static_ui_Ungroup_mid_Ellipse->start_y = 540;
    ui_static_ui_Ungroup_mid_Ellipse->width = 1;
    ui_static_ui_Ungroup_mid_Ellipse->rx = 10;
    ui_static_ui_Ungroup_mid_Ellipse->ry = 10;

    ui_static_ui_Ungroup_left_line->figure_type = 0;
    ui_static_ui_Ungroup_left_line->operate_type = 1;
    ui_static_ui_Ungroup_left_line->layer = 0;
    ui_static_ui_Ungroup_left_line->color = 2;
    ui_static_ui_Ungroup_left_line->start_x = 450;
    ui_static_ui_Ungroup_left_line->start_y = 150;
    ui_static_ui_Ungroup_left_line->width = 1;
    ui_static_ui_Ungroup_left_line->end_x = 600;
    ui_static_ui_Ungroup_left_line->end_y = 380;

    ui_static_ui_Ungroup_right_line->figure_type = 0;
    ui_static_ui_Ungroup_right_line->operate_type = 1;
    ui_static_ui_Ungroup_right_line->layer = 0;
    ui_static_ui_Ungroup_right_line->color = 2;
    ui_static_ui_Ungroup_right_line->start_x = 1470;
    ui_static_ui_Ungroup_right_line->start_y = 150;
    ui_static_ui_Ungroup_right_line->width = 1;
    ui_static_ui_Ungroup_right_line->end_x = 1320;
    ui_static_ui_Ungroup_right_line->end_y = 380;

    ui_static_ui_Ungroup_upArc->figure_type = 4;
    ui_static_ui_Ungroup_upArc->operate_type = 1;
    ui_static_ui_Ungroup_upArc->layer = 0;
    ui_static_ui_Ungroup_upArc->color = 8;
    ui_static_ui_Ungroup_upArc->start_x = 960;
    ui_static_ui_Ungroup_upArc->start_y = 575;
    ui_static_ui_Ungroup_upArc->width = 1;
    ui_static_ui_Ungroup_upArc->start_angle = 340;
    ui_static_ui_Ungroup_upArc->end_angle = 20;
    ui_static_ui_Ungroup_upArc->rx = 300;
    ui_static_ui_Ungroup_upArc->ry = 300;


    ui_proc_5_frame(&ui_static_ui_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_2, sizeof(ui_static_ui_Ungroup_2));
}

void _ui_update_static_ui_Ungroup_2() {
    for (int i = 0; i < 4; i++) {
        ui_static_ui_Ungroup_2.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_static_ui_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_2, sizeof(ui_static_ui_Ungroup_2));
}

void _ui_remove_static_ui_Ungroup_2() {
    for (int i = 0; i < 4; i++) {
        ui_static_ui_Ungroup_2.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_static_ui_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_2, sizeof(ui_static_ui_Ungroup_2));
}

ui_string_frame_t ui_static_ui_Ungroup_3;
ui_interface_string_t* ui_static_ui_Ungroup_speed = &(ui_static_ui_Ungroup_3.option);

void _ui_init_static_ui_Ungroup_3() {
    ui_static_ui_Ungroup_3.option.figure_name[0] = 3;
    ui_static_ui_Ungroup_3.option.figure_name[1] = 0;
    ui_static_ui_Ungroup_3.option.figure_name[2] = 18;
    ui_static_ui_Ungroup_3.option.operate_type = 1;

    ui_static_ui_Ungroup_speed->figure_type = 7;
    ui_static_ui_Ungroup_speed->operate_type = 1;
    ui_static_ui_Ungroup_speed->layer = 0;
    ui_static_ui_Ungroup_speed->color = 0;
    ui_static_ui_Ungroup_speed->start_x = 1364;
    ui_static_ui_Ungroup_speed->start_y = 614;
    ui_static_ui_Ungroup_speed->width = 2;
    ui_static_ui_Ungroup_speed->font_size = 20;
    ui_static_ui_Ungroup_speed->str_length = 5;
    strcpy(ui_static_ui_Ungroup_speed->string, "SPEED");


    ui_proc_string_frame(&ui_static_ui_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_3, sizeof(ui_static_ui_Ungroup_3));
}

void _ui_update_static_ui_Ungroup_3() {
    ui_static_ui_Ungroup_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_static_ui_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_3, sizeof(ui_static_ui_Ungroup_3));
}

void _ui_remove_static_ui_Ungroup_3() {
    ui_static_ui_Ungroup_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_static_ui_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_3, sizeof(ui_static_ui_Ungroup_3));
}
ui_string_frame_t ui_static_ui_Ungroup_4;
ui_interface_string_t* ui_static_ui_Ungroup_CHASSIS_NO_FOLLOW_GIMBAL = &(ui_static_ui_Ungroup_4.option);

void _ui_init_static_ui_Ungroup_4() {
    ui_static_ui_Ungroup_4.option.figure_name[0] = 3;
    ui_static_ui_Ungroup_4.option.figure_name[1] = 0;
    ui_static_ui_Ungroup_4.option.figure_name[2] = 19;
    ui_static_ui_Ungroup_4.option.operate_type = 1;

    ui_static_ui_Ungroup_CHASSIS_NO_FOLLOW_GIMBAL->figure_type = 7;
    ui_static_ui_Ungroup_CHASSIS_NO_FOLLOW_GIMBAL->operate_type = 1;
    ui_static_ui_Ungroup_CHASSIS_NO_FOLLOW_GIMBAL->layer = 0;
    ui_static_ui_Ungroup_CHASSIS_NO_FOLLOW_GIMBAL->color = 0;
    ui_static_ui_Ungroup_CHASSIS_NO_FOLLOW_GIMBAL->start_x = 62;
    ui_static_ui_Ungroup_CHASSIS_NO_FOLLOW_GIMBAL->start_y = 810;
    ui_static_ui_Ungroup_CHASSIS_NO_FOLLOW_GIMBAL->width = 2;
    ui_static_ui_Ungroup_CHASSIS_NO_FOLLOW_GIMBAL->font_size = 20;
    ui_static_ui_Ungroup_CHASSIS_NO_FOLLOW_GIMBAL->str_length = 16;
    strcpy(ui_static_ui_Ungroup_CHASSIS_NO_FOLLOW_GIMBAL->string, "NO_FOLLOW_GIMBAL");


    ui_proc_string_frame(&ui_static_ui_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_4, sizeof(ui_static_ui_Ungroup_4));
}

void _ui_update_static_ui_Ungroup_4() {
    ui_static_ui_Ungroup_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_static_ui_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_4, sizeof(ui_static_ui_Ungroup_4));
}

void _ui_remove_static_ui_Ungroup_4() {
    ui_static_ui_Ungroup_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_static_ui_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_4, sizeof(ui_static_ui_Ungroup_4));
}
ui_string_frame_t ui_static_ui_Ungroup_5;
ui_interface_string_t* ui_static_ui_Ungroup_CHASSIS_FOLLOW_GIMBAL = &(ui_static_ui_Ungroup_5.option);

void _ui_init_static_ui_Ungroup_5() {
    ui_static_ui_Ungroup_5.option.figure_name[0] = 3;
    ui_static_ui_Ungroup_5.option.figure_name[1] = 0;
    ui_static_ui_Ungroup_5.option.figure_name[2] = 20;
    ui_static_ui_Ungroup_5.option.operate_type = 1;

    ui_static_ui_Ungroup_CHASSIS_FOLLOW_GIMBAL->figure_type = 7;
    ui_static_ui_Ungroup_CHASSIS_FOLLOW_GIMBAL->operate_type = 1;
    ui_static_ui_Ungroup_CHASSIS_FOLLOW_GIMBAL->layer = 0;
    ui_static_ui_Ungroup_CHASSIS_FOLLOW_GIMBAL->color = 0;
    ui_static_ui_Ungroup_CHASSIS_FOLLOW_GIMBAL->start_x = 62;
    ui_static_ui_Ungroup_CHASSIS_FOLLOW_GIMBAL->start_y = 765;
    ui_static_ui_Ungroup_CHASSIS_FOLLOW_GIMBAL->width = 2;
    ui_static_ui_Ungroup_CHASSIS_FOLLOW_GIMBAL->font_size = 20;
    ui_static_ui_Ungroup_CHASSIS_FOLLOW_GIMBAL->str_length = 13;
    strcpy(ui_static_ui_Ungroup_CHASSIS_FOLLOW_GIMBAL->string, "FOLLOW_GIMBAL");


    ui_proc_string_frame(&ui_static_ui_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_5, sizeof(ui_static_ui_Ungroup_5));
}

void _ui_update_static_ui_Ungroup_5() {
    ui_static_ui_Ungroup_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_static_ui_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_5, sizeof(ui_static_ui_Ungroup_5));
}

void _ui_remove_static_ui_Ungroup_5() {
    ui_static_ui_Ungroup_5.option.operate_type = 3;

    ui_proc_string_frame(&ui_static_ui_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_5, sizeof(ui_static_ui_Ungroup_5));
}
ui_string_frame_t ui_static_ui_Ungroup_6;
ui_interface_string_t* ui_static_ui_Ungroup_CHASSIS_TOP = &(ui_static_ui_Ungroup_6.option);

void _ui_init_static_ui_Ungroup_6() {
    ui_static_ui_Ungroup_6.option.figure_name[0] = 3;
    ui_static_ui_Ungroup_6.option.figure_name[1] = 0;
    ui_static_ui_Ungroup_6.option.figure_name[2] = 21;
    ui_static_ui_Ungroup_6.option.operate_type = 1;

    ui_static_ui_Ungroup_CHASSIS_TOP->figure_type = 7;
    ui_static_ui_Ungroup_CHASSIS_TOP->operate_type = 1;
    ui_static_ui_Ungroup_CHASSIS_TOP->layer = 0;
    ui_static_ui_Ungroup_CHASSIS_TOP->color = 0;
    ui_static_ui_Ungroup_CHASSIS_TOP->start_x = 62;
    ui_static_ui_Ungroup_CHASSIS_TOP->start_y = 720;
    ui_static_ui_Ungroup_CHASSIS_TOP->width = 2;
    ui_static_ui_Ungroup_CHASSIS_TOP->font_size = 20;
    ui_static_ui_Ungroup_CHASSIS_TOP->str_length = 11;
    strcpy(ui_static_ui_Ungroup_CHASSIS_TOP->string, "CHASSIS_TOP");


    ui_proc_string_frame(&ui_static_ui_Ungroup_6);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_6, sizeof(ui_static_ui_Ungroup_6));
}

void _ui_update_static_ui_Ungroup_6() {
    ui_static_ui_Ungroup_6.option.operate_type = 2;

    ui_proc_string_frame(&ui_static_ui_Ungroup_6);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_6, sizeof(ui_static_ui_Ungroup_6));
}

void _ui_remove_static_ui_Ungroup_6() {
    ui_static_ui_Ungroup_6.option.operate_type = 3;

    ui_proc_string_frame(&ui_static_ui_Ungroup_6);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_6, sizeof(ui_static_ui_Ungroup_6));
}
ui_string_frame_t ui_static_ui_Ungroup_7;
ui_interface_string_t* ui_static_ui_Ungroup_SHOOT_STOP = &(ui_static_ui_Ungroup_7.option);

void _ui_init_static_ui_Ungroup_7() {
    ui_static_ui_Ungroup_7.option.figure_name[0] = 3;
    ui_static_ui_Ungroup_7.option.figure_name[1] = 0;
    ui_static_ui_Ungroup_7.option.figure_name[2] = 22;
    ui_static_ui_Ungroup_7.option.operate_type = 1;

    ui_static_ui_Ungroup_SHOOT_STOP->figure_type = 7;
    ui_static_ui_Ungroup_SHOOT_STOP->operate_type = 1;
    ui_static_ui_Ungroup_SHOOT_STOP->layer = 0;
    ui_static_ui_Ungroup_SHOOT_STOP->color = 0;
    ui_static_ui_Ungroup_SHOOT_STOP->start_x = 62;
    ui_static_ui_Ungroup_SHOOT_STOP->start_y = 600;
    ui_static_ui_Ungroup_SHOOT_STOP->width = 2;
    ui_static_ui_Ungroup_SHOOT_STOP->font_size = 20;
    ui_static_ui_Ungroup_SHOOT_STOP->str_length = 10;
    strcpy(ui_static_ui_Ungroup_SHOOT_STOP->string, "SHOOT_STOP");


    ui_proc_string_frame(&ui_static_ui_Ungroup_7);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_7, sizeof(ui_static_ui_Ungroup_7));
}

void _ui_update_static_ui_Ungroup_7() {
    ui_static_ui_Ungroup_7.option.operate_type = 2;

    ui_proc_string_frame(&ui_static_ui_Ungroup_7);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_7, sizeof(ui_static_ui_Ungroup_7));
}

void _ui_remove_static_ui_Ungroup_7() {
    ui_static_ui_Ungroup_7.option.operate_type = 3;

    ui_proc_string_frame(&ui_static_ui_Ungroup_7);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_7, sizeof(ui_static_ui_Ungroup_7));
}
ui_string_frame_t ui_static_ui_Ungroup_8;
ui_interface_string_t* ui_static_ui_Ungroup_SHOOT_READY = &(ui_static_ui_Ungroup_8.option);

void _ui_init_static_ui_Ungroup_8() {
    ui_static_ui_Ungroup_8.option.figure_name[0] = 3;
    ui_static_ui_Ungroup_8.option.figure_name[1] = 0;
    ui_static_ui_Ungroup_8.option.figure_name[2] = 23;
    ui_static_ui_Ungroup_8.option.operate_type = 1;

    ui_static_ui_Ungroup_SHOOT_READY->figure_type = 7;
    ui_static_ui_Ungroup_SHOOT_READY->operate_type = 1;
    ui_static_ui_Ungroup_SHOOT_READY->layer = 0;
    ui_static_ui_Ungroup_SHOOT_READY->color = 0;
    ui_static_ui_Ungroup_SHOOT_READY->start_x = 62;
    ui_static_ui_Ungroup_SHOOT_READY->start_y = 555;
    ui_static_ui_Ungroup_SHOOT_READY->width = 2;
    ui_static_ui_Ungroup_SHOOT_READY->font_size = 20;
    ui_static_ui_Ungroup_SHOOT_READY->str_length = 11;
    strcpy(ui_static_ui_Ungroup_SHOOT_READY->string, "SHOOT_READY");


    ui_proc_string_frame(&ui_static_ui_Ungroup_8);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_8, sizeof(ui_static_ui_Ungroup_8));
}

void _ui_update_static_ui_Ungroup_8() {
    ui_static_ui_Ungroup_8.option.operate_type = 2;

    ui_proc_string_frame(&ui_static_ui_Ungroup_8);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_8, sizeof(ui_static_ui_Ungroup_8));
}

void _ui_remove_static_ui_Ungroup_8() {
    ui_static_ui_Ungroup_8.option.operate_type = 3;

    ui_proc_string_frame(&ui_static_ui_Ungroup_8);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_8, sizeof(ui_static_ui_Ungroup_8));
}
ui_string_frame_t ui_static_ui_Ungroup_9;
ui_interface_string_t* ui_static_ui_Ungroup_SHOOT_BULLET = &(ui_static_ui_Ungroup_9.option);

void _ui_init_static_ui_Ungroup_9() {
    ui_static_ui_Ungroup_9.option.figure_name[0] = 3;
    ui_static_ui_Ungroup_9.option.figure_name[1] = 0;
    ui_static_ui_Ungroup_9.option.figure_name[2] = 24;
    ui_static_ui_Ungroup_9.option.operate_type = 1;

    ui_static_ui_Ungroup_SHOOT_BULLET->figure_type = 7;
    ui_static_ui_Ungroup_SHOOT_BULLET->operate_type = 1;
    ui_static_ui_Ungroup_SHOOT_BULLET->layer = 0;
    ui_static_ui_Ungroup_SHOOT_BULLET->color = 0;
    ui_static_ui_Ungroup_SHOOT_BULLET->start_x = 62;
    ui_static_ui_Ungroup_SHOOT_BULLET->start_y = 510;
    ui_static_ui_Ungroup_SHOOT_BULLET->width = 2;
    ui_static_ui_Ungroup_SHOOT_BULLET->font_size = 20;
    ui_static_ui_Ungroup_SHOOT_BULLET->str_length = 12;
    strcpy(ui_static_ui_Ungroup_SHOOT_BULLET->string, "SHOOT_BULLET");


    ui_proc_string_frame(&ui_static_ui_Ungroup_9);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_9, sizeof(ui_static_ui_Ungroup_9));
}

void _ui_update_static_ui_Ungroup_9() {
    ui_static_ui_Ungroup_9.option.operate_type = 2;

    ui_proc_string_frame(&ui_static_ui_Ungroup_9);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_9, sizeof(ui_static_ui_Ungroup_9));
}

void _ui_remove_static_ui_Ungroup_9() {
    ui_static_ui_Ungroup_9.option.operate_type = 3;

    ui_proc_string_frame(&ui_static_ui_Ungroup_9);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_9, sizeof(ui_static_ui_Ungroup_9));
}

void ui_init_static_ui_Ungroup() {
    _ui_init_static_ui_Ungroup_0();
    _ui_init_static_ui_Ungroup_1();
    _ui_init_static_ui_Ungroup_2();
    _ui_init_static_ui_Ungroup_3();
    _ui_init_static_ui_Ungroup_4();
    _ui_init_static_ui_Ungroup_5();
    _ui_init_static_ui_Ungroup_6();
    _ui_init_static_ui_Ungroup_7();
    _ui_init_static_ui_Ungroup_8();
    _ui_init_static_ui_Ungroup_9();
}

void ui_update_static_ui_Ungroup() {
    _ui_update_static_ui_Ungroup_0();
    _ui_update_static_ui_Ungroup_1();
    _ui_update_static_ui_Ungroup_2();
    _ui_update_static_ui_Ungroup_3();
    _ui_update_static_ui_Ungroup_4();
    _ui_update_static_ui_Ungroup_5();
    _ui_update_static_ui_Ungroup_6();
    _ui_update_static_ui_Ungroup_7();
    _ui_update_static_ui_Ungroup_8();
    _ui_update_static_ui_Ungroup_9();
}

void ui_remove_static_ui_Ungroup() {
    _ui_remove_static_ui_Ungroup_0();
    _ui_remove_static_ui_Ungroup_1();
    _ui_remove_static_ui_Ungroup_2();
    _ui_remove_static_ui_Ungroup_3();
    _ui_remove_static_ui_Ungroup_4();
    _ui_remove_static_ui_Ungroup_5();
    _ui_remove_static_ui_Ungroup_6();
    _ui_remove_static_ui_Ungroup_7();
    _ui_remove_static_ui_Ungroup_8();
    _ui_remove_static_ui_Ungroup_9();
}

