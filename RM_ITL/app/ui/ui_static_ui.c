//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_7_frame_t ui_static_ui_Ungroup_0;

ui_interface_line_t *ui_static_ui_Ungroup_mid_line = (ui_interface_line_t*)&(ui_static_ui_Ungroup_0.data[0]);
ui_interface_line_t *ui_static_ui_Ungroup_vertical_Line = (ui_interface_line_t*)&(ui_static_ui_Ungroup_0.data[1]);
ui_interface_rect_t *ui_static_ui_Ungroup_speed_mode1 = (ui_interface_rect_t*)&(ui_static_ui_Ungroup_0.data[2]);
ui_interface_rect_t *ui_static_ui_Ungroup_speed_mode2 = (ui_interface_rect_t*)&(ui_static_ui_Ungroup_0.data[3]);
ui_interface_rect_t *ui_static_ui_Ungroup_speed_mode3 = (ui_interface_rect_t*)&(ui_static_ui_Ungroup_0.data[4]);
ui_interface_line_t *ui_static_ui_Ungroup_mid_line1 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_0.data[5]);
ui_interface_arc_t *ui_static_ui_Ungroup_energy_arc1 = (ui_interface_arc_t*)&(ui_static_ui_Ungroup_0.data[6]);

void _ui_init_static_ui_Ungroup_0() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_0.data[i].figure_name[0] = 2;
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
    ui_static_ui_Ungroup_mid_line->end_x = 1161;
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

    ui_static_ui_Ungroup_speed_mode1->figure_type = 1;
    ui_static_ui_Ungroup_speed_mode1->operate_type = 1;
    ui_static_ui_Ungroup_speed_mode1->layer = 0;
    ui_static_ui_Ungroup_speed_mode1->color = 8;
    ui_static_ui_Ungroup_speed_mode1->start_x = 1400;
    ui_static_ui_Ungroup_speed_mode1->start_y = 600;
    ui_static_ui_Ungroup_speed_mode1->width = 1;
    ui_static_ui_Ungroup_speed_mode1->end_x = 1430;
    ui_static_ui_Ungroup_speed_mode1->end_y = 660;

    ui_static_ui_Ungroup_speed_mode2->figure_type = 1;
    ui_static_ui_Ungroup_speed_mode2->operate_type = 1;
    ui_static_ui_Ungroup_speed_mode2->layer = 0;
    ui_static_ui_Ungroup_speed_mode2->color = 8;
    ui_static_ui_Ungroup_speed_mode2->start_x = 1400;
    ui_static_ui_Ungroup_speed_mode2->start_y = 660;
    ui_static_ui_Ungroup_speed_mode2->width = 1;
    ui_static_ui_Ungroup_speed_mode2->end_x = 1430;
    ui_static_ui_Ungroup_speed_mode2->end_y = 720;

    ui_static_ui_Ungroup_speed_mode3->figure_type = 1;
    ui_static_ui_Ungroup_speed_mode3->operate_type = 1;
    ui_static_ui_Ungroup_speed_mode3->layer = 0;
    ui_static_ui_Ungroup_speed_mode3->color = 8;
    ui_static_ui_Ungroup_speed_mode3->start_x = 1400;
    ui_static_ui_Ungroup_speed_mode3->start_y = 720;
    ui_static_ui_Ungroup_speed_mode3->width = 1;
    ui_static_ui_Ungroup_speed_mode3->end_x = 1430;
    ui_static_ui_Ungroup_speed_mode3->end_y = 780;

    ui_static_ui_Ungroup_mid_line1->figure_type = 0;
    ui_static_ui_Ungroup_mid_line1->operate_type = 1;
    ui_static_ui_Ungroup_mid_line1->layer = 0;
    ui_static_ui_Ungroup_mid_line1->color = 8;
    ui_static_ui_Ungroup_mid_line1->start_x = 920;
    ui_static_ui_Ungroup_mid_line1->start_y = 520;
    ui_static_ui_Ungroup_mid_line1->width = 1;
    ui_static_ui_Ungroup_mid_line1->end_x = 1000;
    ui_static_ui_Ungroup_mid_line1->end_y = 520;

    ui_static_ui_Ungroup_energy_arc1->figure_type = 4;
    ui_static_ui_Ungroup_energy_arc1->operate_type = 1;
    ui_static_ui_Ungroup_energy_arc1->layer = 0;
    ui_static_ui_Ungroup_energy_arc1->color = 8;
    ui_static_ui_Ungroup_energy_arc1->start_x = 950;
    ui_static_ui_Ungroup_energy_arc1->start_y = 538;
    ui_static_ui_Ungroup_energy_arc1->width = 1;
    ui_static_ui_Ungroup_energy_arc1->start_angle = 230;
    ui_static_ui_Ungroup_energy_arc1->end_angle = 270;
    ui_static_ui_Ungroup_energy_arc1->rx = 370;
    ui_static_ui_Ungroup_energy_arc1->ry = 370;


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

ui_interface_arc_t *ui_static_ui_Ungroup_energy_arc2 = (ui_interface_arc_t*)&(ui_static_ui_Ungroup_1.data[0]);
ui_interface_line_t *ui_static_ui_Ungroup_energy_line1 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_1.data[1]);
ui_interface_line_t *ui_static_ui_Ungroup_energy_line2 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_1.data[2]);
ui_interface_line_t *ui_static_ui_Ungroup_angle_line1 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_1.data[3]);
ui_interface_line_t *ui_static_ui_Ungroup_angle_line2 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_1.data[4]);
ui_interface_line_t *ui_static_ui_Ungroup_mid_line2 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_1.data[5]);
ui_interface_line_t *ui_static_ui_Ungroup_angle_line3 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_1.data[6]);

void _ui_init_static_ui_Ungroup_1() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_1.data[i].figure_name[0] = 2;
        ui_static_ui_Ungroup_1.data[i].figure_name[1] = 0;
        ui_static_ui_Ungroup_1.data[i].figure_name[2] = i + 7;
        ui_static_ui_Ungroup_1.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_static_ui_Ungroup_1.data[i].operate_type = 0;
    }

    ui_static_ui_Ungroup_energy_arc2->figure_type = 4;
    ui_static_ui_Ungroup_energy_arc2->operate_type = 1;
    ui_static_ui_Ungroup_energy_arc2->layer = 0;
    ui_static_ui_Ungroup_energy_arc2->color = 8;
    ui_static_ui_Ungroup_energy_arc2->start_x = 966;
    ui_static_ui_Ungroup_energy_arc2->start_y = 540;
    ui_static_ui_Ungroup_energy_arc2->width = 1;
    ui_static_ui_Ungroup_energy_arc2->start_angle = 232;
    ui_static_ui_Ungroup_energy_arc2->end_angle = 269;
    ui_static_ui_Ungroup_energy_arc2->rx = 358;
    ui_static_ui_Ungroup_energy_arc2->ry = 360;

    ui_static_ui_Ungroup_energy_line1->figure_type = 0;
    ui_static_ui_Ungroup_energy_line1->operate_type = 1;
    ui_static_ui_Ungroup_energy_line1->layer = 0;
    ui_static_ui_Ungroup_energy_line1->color = 8;
    ui_static_ui_Ungroup_energy_line1->start_x = 678;
    ui_static_ui_Ungroup_energy_line1->start_y = 312;
    ui_static_ui_Ungroup_energy_line1->width = 1;
    ui_static_ui_Ungroup_energy_line1->end_x = 667;
    ui_static_ui_Ungroup_energy_line1->end_y = 300;

    ui_static_ui_Ungroup_energy_line2->figure_type = 0;
    ui_static_ui_Ungroup_energy_line2->operate_type = 1;
    ui_static_ui_Ungroup_energy_line2->layer = 0;
    ui_static_ui_Ungroup_energy_line2->color = 8;
    ui_static_ui_Ungroup_energy_line2->start_x = 579;
    ui_static_ui_Ungroup_energy_line2->start_y = 533;
    ui_static_ui_Ungroup_energy_line2->width = 1;
    ui_static_ui_Ungroup_energy_line2->end_x = 610;
    ui_static_ui_Ungroup_energy_line2->end_y = 533;

    ui_static_ui_Ungroup_angle_line1->figure_type = 0;
    ui_static_ui_Ungroup_angle_line1->operate_type = 1;
    ui_static_ui_Ungroup_angle_line1->layer = 0;
    ui_static_ui_Ungroup_angle_line1->color = 8;
    ui_static_ui_Ungroup_angle_line1->start_x = 1329;
    ui_static_ui_Ungroup_angle_line1->start_y = 540;
    ui_static_ui_Ungroup_angle_line1->width = 5;
    ui_static_ui_Ungroup_angle_line1->end_x = 1354;
    ui_static_ui_Ungroup_angle_line1->end_y = 540;

    ui_static_ui_Ungroup_angle_line2->figure_type = 0;
    ui_static_ui_Ungroup_angle_line2->operate_type = 1;
    ui_static_ui_Ungroup_angle_line2->layer = 0;
    ui_static_ui_Ungroup_angle_line2->color = 8;
    ui_static_ui_Ungroup_angle_line2->start_x = 1230;
    ui_static_ui_Ungroup_angle_line2->start_y = 810;
    ui_static_ui_Ungroup_angle_line2->width = 5;
    ui_static_ui_Ungroup_angle_line2->end_x = 1250;
    ui_static_ui_Ungroup_angle_line2->end_y = 810;

    ui_static_ui_Ungroup_mid_line2->figure_type = 0;
    ui_static_ui_Ungroup_mid_line2->operate_type = 1;
    ui_static_ui_Ungroup_mid_line2->layer = 0;
    ui_static_ui_Ungroup_mid_line2->color = 8;
    ui_static_ui_Ungroup_mid_line2->start_x = 900;
    ui_static_ui_Ungroup_mid_line2->start_y = 500;
    ui_static_ui_Ungroup_mid_line2->width = 1;
    ui_static_ui_Ungroup_mid_line2->end_x = 1020;
    ui_static_ui_Ungroup_mid_line2->end_y = 500;

    ui_static_ui_Ungroup_angle_line3->figure_type = 0;
    ui_static_ui_Ungroup_angle_line3->operate_type = 1;
    ui_static_ui_Ungroup_angle_line3->layer = 0;
    ui_static_ui_Ungroup_angle_line3->color = 8;
    ui_static_ui_Ungroup_angle_line3->start_x = 1230;
    ui_static_ui_Ungroup_angle_line3->start_y = 270;
    ui_static_ui_Ungroup_angle_line3->width = 5;
    ui_static_ui_Ungroup_angle_line3->end_x = 1250;
    ui_static_ui_Ungroup_angle_line3->end_y = 270;


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
ui_7_frame_t ui_static_ui_Ungroup_2;

ui_interface_line_t *ui_static_ui_Ungroup_angle_line4 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_2.data[0]);
ui_interface_line_t *ui_static_ui_Ungroup_angle_line5 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_2.data[1]);
ui_interface_line_t *ui_static_ui_Ungroup_angle_line6 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_2.data[2]);
ui_interface_line_t *ui_static_ui_Ungroup_angle_line7 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_2.data[3]);
ui_interface_line_t *ui_static_ui_Ungroup_angle_line8 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_2.data[4]);
ui_interface_line_t *ui_static_ui_Ungroup_angle_line9 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_2.data[5]);
ui_interface_line_t *ui_static_ui_Ungroup_angle_line10 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_2.data[6]);

void _ui_init_static_ui_Ungroup_2() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_2.data[i].figure_name[0] = 2;
        ui_static_ui_Ungroup_2.data[i].figure_name[1] = 0;
        ui_static_ui_Ungroup_2.data[i].figure_name[2] = i + 14;
        ui_static_ui_Ungroup_2.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_static_ui_Ungroup_2.data[i].operate_type = 0;
    }

    ui_static_ui_Ungroup_angle_line4->figure_type = 0;
    ui_static_ui_Ungroup_angle_line4->operate_type = 1;
    ui_static_ui_Ungroup_angle_line4->layer = 0;
    ui_static_ui_Ungroup_angle_line4->color = 8;
    ui_static_ui_Ungroup_angle_line4->start_x = 1260;
    ui_static_ui_Ungroup_angle_line4->start_y = 314;
    ui_static_ui_Ungroup_angle_line4->width = 5;
    ui_static_ui_Ungroup_angle_line4->end_x = 1285;
    ui_static_ui_Ungroup_angle_line4->end_y = 314;

    ui_static_ui_Ungroup_angle_line5->figure_type = 0;
    ui_static_ui_Ungroup_angle_line5->operate_type = 1;
    ui_static_ui_Ungroup_angle_line5->layer = 0;
    ui_static_ui_Ungroup_angle_line5->color = 8;
    ui_static_ui_Ungroup_angle_line5->start_x = 1319;
    ui_static_ui_Ungroup_angle_line5->start_y = 451;
    ui_static_ui_Ungroup_angle_line5->width = 5;
    ui_static_ui_Ungroup_angle_line5->end_x = 1344;
    ui_static_ui_Ungroup_angle_line5->end_y = 451;

    ui_static_ui_Ungroup_angle_line6->figure_type = 0;
    ui_static_ui_Ungroup_angle_line6->operate_type = 1;
    ui_static_ui_Ungroup_angle_line6->layer = 0;
    ui_static_ui_Ungroup_angle_line6->color = 8;
    ui_static_ui_Ungroup_angle_line6->start_x = 1282;
    ui_static_ui_Ungroup_angle_line6->start_y = 359;
    ui_static_ui_Ungroup_angle_line6->width = 5;
    ui_static_ui_Ungroup_angle_line6->end_x = 1307;
    ui_static_ui_Ungroup_angle_line6->end_y = 359;

    ui_static_ui_Ungroup_angle_line7->figure_type = 0;
    ui_static_ui_Ungroup_angle_line7->operate_type = 1;
    ui_static_ui_Ungroup_angle_line7->layer = 0;
    ui_static_ui_Ungroup_angle_line7->color = 8;
    ui_static_ui_Ungroup_angle_line7->start_x = 1304;
    ui_static_ui_Ungroup_angle_line7->start_y = 408;
    ui_static_ui_Ungroup_angle_line7->width = 5;
    ui_static_ui_Ungroup_angle_line7->end_x = 1327;
    ui_static_ui_Ungroup_angle_line7->end_y = 408;

    ui_static_ui_Ungroup_angle_line8->figure_type = 0;
    ui_static_ui_Ungroup_angle_line8->operate_type = 1;
    ui_static_ui_Ungroup_angle_line8->layer = 0;
    ui_static_ui_Ungroup_angle_line8->color = 8;
    ui_static_ui_Ungroup_angle_line8->start_x = 1326;
    ui_static_ui_Ungroup_angle_line8->start_y = 493;
    ui_static_ui_Ungroup_angle_line8->width = 5;
    ui_static_ui_Ungroup_angle_line8->end_x = 1349;
    ui_static_ui_Ungroup_angle_line8->end_y = 493;

    ui_static_ui_Ungroup_angle_line9->figure_type = 0;
    ui_static_ui_Ungroup_angle_line9->operate_type = 1;
    ui_static_ui_Ungroup_angle_line9->layer = 0;
    ui_static_ui_Ungroup_angle_line9->color = 8;
    ui_static_ui_Ungroup_angle_line9->start_x = 1327;
    ui_static_ui_Ungroup_angle_line9->start_y = 586;
    ui_static_ui_Ungroup_angle_line9->width = 5;
    ui_static_ui_Ungroup_angle_line9->end_x = 1352;
    ui_static_ui_Ungroup_angle_line9->end_y = 586;

    ui_static_ui_Ungroup_angle_line10->figure_type = 0;
    ui_static_ui_Ungroup_angle_line10->operate_type = 1;
    ui_static_ui_Ungroup_angle_line10->layer = 0;
    ui_static_ui_Ungroup_angle_line10->color = 8;
    ui_static_ui_Ungroup_angle_line10->start_x = 1317;
    ui_static_ui_Ungroup_angle_line10->start_y = 632;
    ui_static_ui_Ungroup_angle_line10->width = 5;
    ui_static_ui_Ungroup_angle_line10->end_x = 1342;
    ui_static_ui_Ungroup_angle_line10->end_y = 632;


    ui_proc_7_frame(&ui_static_ui_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_2, sizeof(ui_static_ui_Ungroup_2));
}

void _ui_update_static_ui_Ungroup_2() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_2.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_static_ui_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_2, sizeof(ui_static_ui_Ungroup_2));
}

void _ui_remove_static_ui_Ungroup_2() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_2.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_static_ui_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_2, sizeof(ui_static_ui_Ungroup_2));
}
ui_7_frame_t ui_static_ui_Ungroup_3;

ui_interface_line_t *ui_static_ui_Ungroup_angle_line11 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_3.data[0]);
ui_interface_line_t *ui_static_ui_Ungroup_angle_line12 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_3.data[1]);
ui_interface_line_t *ui_static_ui_Ungroup_mid_line3 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_3.data[2]);
ui_interface_line_t *ui_static_ui_Ungroup_angle_line13 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_3.data[3]);
ui_interface_number_t *ui_static_ui_Ungroup_angle_0 = (ui_interface_number_t*)&(ui_static_ui_Ungroup_3.data[4]);
ui_interface_number_t *ui_static_ui_Ungroup_angle_5 = (ui_interface_number_t*)&(ui_static_ui_Ungroup_3.data[5]);
ui_interface_number_t *ui_static_ui_Ungroup_angle_10 = (ui_interface_number_t*)&(ui_static_ui_Ungroup_3.data[6]);

void _ui_init_static_ui_Ungroup_3() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_3.data[i].figure_name[0] = 2;
        ui_static_ui_Ungroup_3.data[i].figure_name[1] = 0;
        ui_static_ui_Ungroup_3.data[i].figure_name[2] = i + 21;
        ui_static_ui_Ungroup_3.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_static_ui_Ungroup_3.data[i].operate_type = 0;
    }

    ui_static_ui_Ungroup_angle_line11->figure_type = 0;
    ui_static_ui_Ungroup_angle_line11->operate_type = 1;
    ui_static_ui_Ungroup_angle_line11->layer = 0;
    ui_static_ui_Ungroup_angle_line11->color = 8;
    ui_static_ui_Ungroup_angle_line11->start_x = 1303;
    ui_static_ui_Ungroup_angle_line11->start_y = 673;
    ui_static_ui_Ungroup_angle_line11->width = 5;
    ui_static_ui_Ungroup_angle_line11->end_x = 1329;
    ui_static_ui_Ungroup_angle_line11->end_y = 673;

    ui_static_ui_Ungroup_angle_line12->figure_type = 0;
    ui_static_ui_Ungroup_angle_line12->operate_type = 1;
    ui_static_ui_Ungroup_angle_line12->layer = 0;
    ui_static_ui_Ungroup_angle_line12->color = 8;
    ui_static_ui_Ungroup_angle_line12->start_x = 1285;
    ui_static_ui_Ungroup_angle_line12->start_y = 720;
    ui_static_ui_Ungroup_angle_line12->width = 5;
    ui_static_ui_Ungroup_angle_line12->end_x = 1311;
    ui_static_ui_Ungroup_angle_line12->end_y = 720;

    ui_static_ui_Ungroup_mid_line3->figure_type = 0;
    ui_static_ui_Ungroup_mid_line3->operate_type = 1;
    ui_static_ui_Ungroup_mid_line3->layer = 0;
    ui_static_ui_Ungroup_mid_line3->color = 8;
    ui_static_ui_Ungroup_mid_line3->start_x = 880;
    ui_static_ui_Ungroup_mid_line3->start_y = 480;
    ui_static_ui_Ungroup_mid_line3->width = 1;
    ui_static_ui_Ungroup_mid_line3->end_x = 1041;
    ui_static_ui_Ungroup_mid_line3->end_y = 480;

    ui_static_ui_Ungroup_angle_line13->figure_type = 0;
    ui_static_ui_Ungroup_angle_line13->operate_type = 1;
    ui_static_ui_Ungroup_angle_line13->layer = 0;
    ui_static_ui_Ungroup_angle_line13->color = 8;
    ui_static_ui_Ungroup_angle_line13->start_x = 1259;
    ui_static_ui_Ungroup_angle_line13->start_y = 765;
    ui_static_ui_Ungroup_angle_line13->width = 5;
    ui_static_ui_Ungroup_angle_line13->end_x = 1282;
    ui_static_ui_Ungroup_angle_line13->end_y = 765;

    ui_static_ui_Ungroup_angle_0->figure_type = 6;
    ui_static_ui_Ungroup_angle_0->operate_type = 1;
    ui_static_ui_Ungroup_angle_0->layer = 0;
    ui_static_ui_Ungroup_angle_0->color = 8;
    ui_static_ui_Ungroup_angle_0->start_x = 1308;
    ui_static_ui_Ungroup_angle_0->start_y = 551;
    ui_static_ui_Ungroup_angle_0->width = 2;
    ui_static_ui_Ungroup_angle_0->font_size = 15;
    ui_static_ui_Ungroup_angle_0->number = 0;

    ui_static_ui_Ungroup_angle_5->figure_type = 6;
    ui_static_ui_Ungroup_angle_5->operate_type = 1;
    ui_static_ui_Ungroup_angle_5->layer = 0;
    ui_static_ui_Ungroup_angle_5->color = 8;
    ui_static_ui_Ungroup_angle_5->start_x = 1309;
    ui_static_ui_Ungroup_angle_5->start_y = 597;
    ui_static_ui_Ungroup_angle_5->width = 2;
    ui_static_ui_Ungroup_angle_5->font_size = 15;
    ui_static_ui_Ungroup_angle_5->number = 5;

    ui_static_ui_Ungroup_angle_10->figure_type = 6;
    ui_static_ui_Ungroup_angle_10->operate_type = 1;
    ui_static_ui_Ungroup_angle_10->layer = 0;
    ui_static_ui_Ungroup_angle_10->color = 8;
    ui_static_ui_Ungroup_angle_10->start_x = 1289;
    ui_static_ui_Ungroup_angle_10->start_y = 641;
    ui_static_ui_Ungroup_angle_10->width = 1;
    ui_static_ui_Ungroup_angle_10->font_size = 13;
    ui_static_ui_Ungroup_angle_10->number = 10;


    ui_proc_7_frame(&ui_static_ui_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_3, sizeof(ui_static_ui_Ungroup_3));
}

void _ui_update_static_ui_Ungroup_3() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_3.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_static_ui_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_3, sizeof(ui_static_ui_Ungroup_3));
}

void _ui_remove_static_ui_Ungroup_3() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_3.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_static_ui_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_3, sizeof(ui_static_ui_Ungroup_3));
}
ui_7_frame_t ui_static_ui_Ungroup_4;

ui_interface_number_t *ui_static_ui_Ungroup_angle_15 = (ui_interface_number_t*)&(ui_static_ui_Ungroup_4.data[0]);
ui_interface_number_t *ui_static_ui_Ungroup_angle_20 = (ui_interface_number_t*)&(ui_static_ui_Ungroup_4.data[1]);
ui_interface_number_t *ui_static_ui_Ungroup_angle_25 = (ui_interface_number_t*)&(ui_static_ui_Ungroup_4.data[2]);
ui_interface_number_t *ui_static_ui_Ungroup_angle_30 = (ui_interface_number_t*)&(ui_static_ui_Ungroup_4.data[3]);
ui_interface_line_t *ui_static_ui_Ungroup_mid_line4 = (ui_interface_line_t*)&(ui_static_ui_Ungroup_4.data[4]);
ui_interface_ellipse_t *ui_static_ui_Ungroup_mid_Ellipse = (ui_interface_ellipse_t*)&(ui_static_ui_Ungroup_4.data[5]);
ui_interface_rect_t *ui_static_ui_Ungroup_chassis = (ui_interface_rect_t*)&(ui_static_ui_Ungroup_4.data[6]);

void _ui_init_static_ui_Ungroup_4() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_4.data[i].figure_name[0] = 2;
        ui_static_ui_Ungroup_4.data[i].figure_name[1] = 0;
        ui_static_ui_Ungroup_4.data[i].figure_name[2] = i + 28;
        ui_static_ui_Ungroup_4.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_static_ui_Ungroup_4.data[i].operate_type = 0;
    }

    ui_static_ui_Ungroup_angle_15->figure_type = 6;
    ui_static_ui_Ungroup_angle_15->operate_type = 1;
    ui_static_ui_Ungroup_angle_15->layer = 0;
    ui_static_ui_Ungroup_angle_15->color = 8;
    ui_static_ui_Ungroup_angle_15->start_x = 1275;
    ui_static_ui_Ungroup_angle_15->start_y = 685;
    ui_static_ui_Ungroup_angle_15->width = 1;
    ui_static_ui_Ungroup_angle_15->font_size = 13;
    ui_static_ui_Ungroup_angle_15->number = 15;

    ui_static_ui_Ungroup_angle_20->figure_type = 6;
    ui_static_ui_Ungroup_angle_20->operate_type = 1;
    ui_static_ui_Ungroup_angle_20->layer = 0;
    ui_static_ui_Ungroup_angle_20->color = 8;
    ui_static_ui_Ungroup_angle_20->start_x = 1257;
    ui_static_ui_Ungroup_angle_20->start_y = 729;
    ui_static_ui_Ungroup_angle_20->width = 1;
    ui_static_ui_Ungroup_angle_20->font_size = 13;
    ui_static_ui_Ungroup_angle_20->number = 20;

    ui_static_ui_Ungroup_angle_25->figure_type = 6;
    ui_static_ui_Ungroup_angle_25->operate_type = 1;
    ui_static_ui_Ungroup_angle_25->layer = 0;
    ui_static_ui_Ungroup_angle_25->color = 8;
    ui_static_ui_Ungroup_angle_25->start_x = 1232;
    ui_static_ui_Ungroup_angle_25->start_y = 774;
    ui_static_ui_Ungroup_angle_25->width = 1;
    ui_static_ui_Ungroup_angle_25->font_size = 13;
    ui_static_ui_Ungroup_angle_25->number = 25;

    ui_static_ui_Ungroup_angle_30->figure_type = 6;
    ui_static_ui_Ungroup_angle_30->operate_type = 1;
    ui_static_ui_Ungroup_angle_30->layer = 0;
    ui_static_ui_Ungroup_angle_30->color = 8;
    ui_static_ui_Ungroup_angle_30->start_x = 1202;
    ui_static_ui_Ungroup_angle_30->start_y = 821;
    ui_static_ui_Ungroup_angle_30->width = 1;
    ui_static_ui_Ungroup_angle_30->font_size = 13;
    ui_static_ui_Ungroup_angle_30->number = 30;

    ui_static_ui_Ungroup_mid_line4->figure_type = 0;
    ui_static_ui_Ungroup_mid_line4->operate_type = 1;
    ui_static_ui_Ungroup_mid_line4->layer = 0;
    ui_static_ui_Ungroup_mid_line4->color = 8;
    ui_static_ui_Ungroup_mid_line4->start_x = 860;
    ui_static_ui_Ungroup_mid_line4->start_y = 460;
    ui_static_ui_Ungroup_mid_line4->width = 1;
    ui_static_ui_Ungroup_mid_line4->end_x = 1060;
    ui_static_ui_Ungroup_mid_line4->end_y = 460;

    ui_static_ui_Ungroup_mid_Ellipse->figure_type = 3;
    ui_static_ui_Ungroup_mid_Ellipse->operate_type = 1;
    ui_static_ui_Ungroup_mid_Ellipse->layer = 0;
    ui_static_ui_Ungroup_mid_Ellipse->color = 8;
    ui_static_ui_Ungroup_mid_Ellipse->start_x = 960;
    ui_static_ui_Ungroup_mid_Ellipse->start_y = 540;
    ui_static_ui_Ungroup_mid_Ellipse->width = 1;
    ui_static_ui_Ungroup_mid_Ellipse->rx = 10;
    ui_static_ui_Ungroup_mid_Ellipse->ry = 10;

    ui_static_ui_Ungroup_chassis->figure_type = 1;
    ui_static_ui_Ungroup_chassis->operate_type = 1;
    ui_static_ui_Ungroup_chassis->layer = 0;
    ui_static_ui_Ungroup_chassis->color = 0;
    ui_static_ui_Ungroup_chassis->start_x = 145;
    ui_static_ui_Ungroup_chassis->start_y = 520;
    ui_static_ui_Ungroup_chassis->width = 2;
    ui_static_ui_Ungroup_chassis->end_x = 345;
    ui_static_ui_Ungroup_chassis->end_y = 720;


    ui_proc_7_frame(&ui_static_ui_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_4, sizeof(ui_static_ui_Ungroup_4));
}

void _ui_update_static_ui_Ungroup_4() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_4.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_static_ui_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_4, sizeof(ui_static_ui_Ungroup_4));
}

void _ui_remove_static_ui_Ungroup_4() {
    for (int i = 0; i < 7; i++) {
        ui_static_ui_Ungroup_4.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_static_ui_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_4, sizeof(ui_static_ui_Ungroup_4));
}
ui_5_frame_t ui_static_ui_Ungroup_5;

ui_interface_number_t *ui_static_ui_Ungroup_power_0 = (ui_interface_number_t*)&(ui_static_ui_Ungroup_5.data[0]);
ui_interface_number_t *ui_static_ui_Ungroup_power_120 = (ui_interface_number_t*)&(ui_static_ui_Ungroup_5.data[1]);
ui_interface_number_t *ui_static_ui_Ungroup_power_60 = (ui_interface_number_t*)&(ui_static_ui_Ungroup_5.data[2]);
ui_interface_line_t *ui_static_ui_Ungroup_left_line = (ui_interface_line_t*)&(ui_static_ui_Ungroup_5.data[3]);
ui_interface_line_t *ui_static_ui_Ungroup_right_line = (ui_interface_line_t*)&(ui_static_ui_Ungroup_5.data[4]);

void _ui_init_static_ui_Ungroup_5() {
    for (int i = 0; i < 5; i++) {
        ui_static_ui_Ungroup_5.data[i].figure_name[0] = 2;
        ui_static_ui_Ungroup_5.data[i].figure_name[1] = 0;
        ui_static_ui_Ungroup_5.data[i].figure_name[2] = i + 35;
        ui_static_ui_Ungroup_5.data[i].operate_type = 1;
    }
    for (int i = 5; i < 5; i++) {
        ui_static_ui_Ungroup_5.data[i].operate_type = 0;
    }

    ui_static_ui_Ungroup_power_0->figure_type = 6;
    ui_static_ui_Ungroup_power_0->operate_type = 1;
    ui_static_ui_Ungroup_power_0->layer = 0;
    ui_static_ui_Ungroup_power_0->color = 2;
    ui_static_ui_Ungroup_power_0->start_x = 675;
    ui_static_ui_Ungroup_power_0->start_y = 300;
    ui_static_ui_Ungroup_power_0->width = 2;
    ui_static_ui_Ungroup_power_0->font_size = 20;
    ui_static_ui_Ungroup_power_0->number = 0;

    ui_static_ui_Ungroup_power_120->figure_type = 6;
    ui_static_ui_Ungroup_power_120->operate_type = 1;
    ui_static_ui_Ungroup_power_120->layer = 0;
    ui_static_ui_Ungroup_power_120->color = 2;
    ui_static_ui_Ungroup_power_120->start_x = 559;
    ui_static_ui_Ungroup_power_120->start_y = 570;
    ui_static_ui_Ungroup_power_120->width = 2;
    ui_static_ui_Ungroup_power_120->font_size = 20;
    ui_static_ui_Ungroup_power_120->number = 120;

    ui_static_ui_Ungroup_power_60->figure_type = 6;
    ui_static_ui_Ungroup_power_60->operate_type = 1;
    ui_static_ui_Ungroup_power_60->layer = 0;
    ui_static_ui_Ungroup_power_60->color = 2;
    ui_static_ui_Ungroup_power_60->start_x = 552;
    ui_static_ui_Ungroup_power_60->start_y = 415;
    ui_static_ui_Ungroup_power_60->width = 2;
    ui_static_ui_Ungroup_power_60->font_size = 20;
    ui_static_ui_Ungroup_power_60->number = 60;

    ui_static_ui_Ungroup_left_line->figure_type = 0;
    ui_static_ui_Ungroup_left_line->operate_type = 1;
    ui_static_ui_Ungroup_left_line->layer = 0;
    ui_static_ui_Ungroup_left_line->color = 2;
    ui_static_ui_Ungroup_left_line->start_x = 440;
    ui_static_ui_Ungroup_left_line->start_y = 230;
    ui_static_ui_Ungroup_left_line->width = 2;
    ui_static_ui_Ungroup_left_line->end_x = 601;
    ui_static_ui_Ungroup_left_line->end_y = 380;

    ui_static_ui_Ungroup_right_line->figure_type = 0;
    ui_static_ui_Ungroup_right_line->operate_type = 1;
    ui_static_ui_Ungroup_right_line->layer = 0;
    ui_static_ui_Ungroup_right_line->color = 2;
    ui_static_ui_Ungroup_right_line->start_x = 1481;
    ui_static_ui_Ungroup_right_line->start_y = 230;
    ui_static_ui_Ungroup_right_line->width = 2;
    ui_static_ui_Ungroup_right_line->end_x = 1320;
    ui_static_ui_Ungroup_right_line->end_y = 377;


    ui_proc_5_frame(&ui_static_ui_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_5, sizeof(ui_static_ui_Ungroup_5));
}

void _ui_update_static_ui_Ungroup_5() {
    for (int i = 0; i < 5; i++) {
        ui_static_ui_Ungroup_5.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_static_ui_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_5, sizeof(ui_static_ui_Ungroup_5));
}

void _ui_remove_static_ui_Ungroup_5() {
    for (int i = 0; i < 5; i++) {
        ui_static_ui_Ungroup_5.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_static_ui_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_5, sizeof(ui_static_ui_Ungroup_5));
}

ui_string_frame_t ui_static_ui_Ungroup_6;
ui_interface_string_t* ui_static_ui_Ungroup_speed = &(ui_static_ui_Ungroup_6.option);

void _ui_init_static_ui_Ungroup_6() {
    ui_static_ui_Ungroup_6.option.figure_name[0] = 2;
    ui_static_ui_Ungroup_6.option.figure_name[1] = 0;
    ui_static_ui_Ungroup_6.option.figure_name[2] = 40;
    ui_static_ui_Ungroup_6.option.operate_type = 1;

    ui_static_ui_Ungroup_speed->figure_type = 7;
    ui_static_ui_Ungroup_speed->operate_type = 1;
    ui_static_ui_Ungroup_speed->layer = 0;
    ui_static_ui_Ungroup_speed->color = 0;
    ui_static_ui_Ungroup_speed->start_x = 1369;
    ui_static_ui_Ungroup_speed->start_y = 812;
    ui_static_ui_Ungroup_speed->width = 2;
    ui_static_ui_Ungroup_speed->font_size = 20;
    ui_static_ui_Ungroup_speed->str_length = 5;
    strcpy(ui_static_ui_Ungroup_speed->string, "SPEED");


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
ui_interface_string_t* ui_static_ui_Ungroup_10_1 = &(ui_static_ui_Ungroup_7.option);

void _ui_init_static_ui_Ungroup_7() {
    ui_static_ui_Ungroup_7.option.figure_name[0] = 2;
    ui_static_ui_Ungroup_7.option.figure_name[1] = 0;
    ui_static_ui_Ungroup_7.option.figure_name[2] = 41;
    ui_static_ui_Ungroup_7.option.operate_type = 1;

    ui_static_ui_Ungroup_10_1->figure_type = 7;
    ui_static_ui_Ungroup_10_1->operate_type = 1;
    ui_static_ui_Ungroup_10_1->layer = 0;
    ui_static_ui_Ungroup_10_1->color = 8;
    ui_static_ui_Ungroup_10_1->start_x = 1279;
    ui_static_ui_Ungroup_10_1->start_y = 460;
    ui_static_ui_Ungroup_10_1->width = 1;
    ui_static_ui_Ungroup_10_1->font_size = 14;
    ui_static_ui_Ungroup_10_1->str_length = 3;
    strcpy(ui_static_ui_Ungroup_10_1->string, "-10");


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
ui_interface_string_t* ui_static_ui_Ungroup_20_1 = &(ui_static_ui_Ungroup_8.option);

void _ui_init_static_ui_Ungroup_8() {
    ui_static_ui_Ungroup_8.option.figure_name[0] = 2;
    ui_static_ui_Ungroup_8.option.figure_name[1] = 0;
    ui_static_ui_Ungroup_8.option.figure_name[2] = 42;
    ui_static_ui_Ungroup_8.option.operate_type = 1;

    ui_static_ui_Ungroup_20_1->figure_type = 7;
    ui_static_ui_Ungroup_20_1->operate_type = 1;
    ui_static_ui_Ungroup_20_1->layer = 0;
    ui_static_ui_Ungroup_20_1->color = 8;
    ui_static_ui_Ungroup_20_1->start_x = 1241;
    ui_static_ui_Ungroup_20_1->start_y = 370;
    ui_static_ui_Ungroup_20_1->width = 1;
    ui_static_ui_Ungroup_20_1->font_size = 13;
    ui_static_ui_Ungroup_20_1->str_length = 3;
    strcpy(ui_static_ui_Ungroup_20_1->string, "-20");


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
ui_interface_string_t* ui_static_ui_Ungroup_25_1 = &(ui_static_ui_Ungroup_9.option);

void _ui_init_static_ui_Ungroup_9() {
    ui_static_ui_Ungroup_9.option.figure_name[0] = 2;
    ui_static_ui_Ungroup_9.option.figure_name[1] = 0;
    ui_static_ui_Ungroup_9.option.figure_name[2] = 43;
    ui_static_ui_Ungroup_9.option.operate_type = 1;

    ui_static_ui_Ungroup_25_1->figure_type = 7;
    ui_static_ui_Ungroup_25_1->operate_type = 1;
    ui_static_ui_Ungroup_25_1->layer = 0;
    ui_static_ui_Ungroup_25_1->color = 8;
    ui_static_ui_Ungroup_25_1->start_x = 1218;
    ui_static_ui_Ungroup_25_1->start_y = 325;
    ui_static_ui_Ungroup_25_1->width = 1;
    ui_static_ui_Ungroup_25_1->font_size = 13;
    ui_static_ui_Ungroup_25_1->str_length = 3;
    strcpy(ui_static_ui_Ungroup_25_1->string, "-25");


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
ui_string_frame_t ui_static_ui_Ungroup_10;
ui_interface_string_t* ui_static_ui_Ungroup_30_1 = &(ui_static_ui_Ungroup_10.option);

void _ui_init_static_ui_Ungroup_10() {
    ui_static_ui_Ungroup_10.option.figure_name[0] = 2;
    ui_static_ui_Ungroup_10.option.figure_name[1] = 0;
    ui_static_ui_Ungroup_10.option.figure_name[2] = 44;
    ui_static_ui_Ungroup_10.option.operate_type = 1;

    ui_static_ui_Ungroup_30_1->figure_type = 7;
    ui_static_ui_Ungroup_30_1->operate_type = 1;
    ui_static_ui_Ungroup_30_1->layer = 0;
    ui_static_ui_Ungroup_30_1->color = 8;
    ui_static_ui_Ungroup_30_1->start_x = 1188;
    ui_static_ui_Ungroup_30_1->start_y = 279;
    ui_static_ui_Ungroup_30_1->width = 1;
    ui_static_ui_Ungroup_30_1->font_size = 13;
    ui_static_ui_Ungroup_30_1->str_length = 3;
    strcpy(ui_static_ui_Ungroup_30_1->string, "-30");


    ui_proc_string_frame(&ui_static_ui_Ungroup_10);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_10, sizeof(ui_static_ui_Ungroup_10));
}

void _ui_update_static_ui_Ungroup_10() {
    ui_static_ui_Ungroup_10.option.operate_type = 2;

    ui_proc_string_frame(&ui_static_ui_Ungroup_10);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_10, sizeof(ui_static_ui_Ungroup_10));
}

void _ui_remove_static_ui_Ungroup_10() {
    ui_static_ui_Ungroup_10.option.operate_type = 3;

    ui_proc_string_frame(&ui_static_ui_Ungroup_10);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_10, sizeof(ui_static_ui_Ungroup_10));
}
ui_string_frame_t ui_static_ui_Ungroup_11;
ui_interface_string_t* ui_static_ui_Ungroup_angle5_1 = &(ui_static_ui_Ungroup_11.option);

void _ui_init_static_ui_Ungroup_11() {
    ui_static_ui_Ungroup_11.option.figure_name[0] = 2;
    ui_static_ui_Ungroup_11.option.figure_name[1] = 0;
    ui_static_ui_Ungroup_11.option.figure_name[2] = 45;
    ui_static_ui_Ungroup_11.option.operate_type = 1;

    ui_static_ui_Ungroup_angle5_1->figure_type = 7;
    ui_static_ui_Ungroup_angle5_1->operate_type = 1;
    ui_static_ui_Ungroup_angle5_1->layer = 0;
    ui_static_ui_Ungroup_angle5_1->color = 8;
    ui_static_ui_Ungroup_angle5_1->start_x = 1293;
    ui_static_ui_Ungroup_angle5_1->start_y = 503;
    ui_static_ui_Ungroup_angle5_1->width = 2;
    ui_static_ui_Ungroup_angle5_1->font_size = 15;
    ui_static_ui_Ungroup_angle5_1->str_length = 2;
    strcpy(ui_static_ui_Ungroup_angle5_1->string, "-5");


    ui_proc_string_frame(&ui_static_ui_Ungroup_11);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_11, sizeof(ui_static_ui_Ungroup_11));
}

void _ui_update_static_ui_Ungroup_11() {
    ui_static_ui_Ungroup_11.option.operate_type = 2;

    ui_proc_string_frame(&ui_static_ui_Ungroup_11);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_11, sizeof(ui_static_ui_Ungroup_11));
}

void _ui_remove_static_ui_Ungroup_11() {
    ui_static_ui_Ungroup_11.option.operate_type = 3;

    ui_proc_string_frame(&ui_static_ui_Ungroup_11);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_11, sizeof(ui_static_ui_Ungroup_11));
}
ui_string_frame_t ui_static_ui_Ungroup_12;
ui_interface_string_t* ui_static_ui_Ungroup_angle_15_1 = &(ui_static_ui_Ungroup_12.option);

void _ui_init_static_ui_Ungroup_12() {
    ui_static_ui_Ungroup_12.option.figure_name[0] = 2;
    ui_static_ui_Ungroup_12.option.figure_name[1] = 0;
    ui_static_ui_Ungroup_12.option.figure_name[2] = 46;
    ui_static_ui_Ungroup_12.option.operate_type = 1;

    ui_static_ui_Ungroup_angle_15_1->figure_type = 7;
    ui_static_ui_Ungroup_angle_15_1->operate_type = 1;
    ui_static_ui_Ungroup_angle_15_1->layer = 0;
    ui_static_ui_Ungroup_angle_15_1->color = 8;
    ui_static_ui_Ungroup_angle_15_1->start_x = 1262;
    ui_static_ui_Ungroup_angle_15_1->start_y = 419;
    ui_static_ui_Ungroup_angle_15_1->width = 1;
    ui_static_ui_Ungroup_angle_15_1->font_size = 13;
    ui_static_ui_Ungroup_angle_15_1->str_length = 3;
    strcpy(ui_static_ui_Ungroup_angle_15_1->string, "-15");


    ui_proc_string_frame(&ui_static_ui_Ungroup_12);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_12, sizeof(ui_static_ui_Ungroup_12));
}

void _ui_update_static_ui_Ungroup_12() {
    ui_static_ui_Ungroup_12.option.operate_type = 2;

    ui_proc_string_frame(&ui_static_ui_Ungroup_12);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_12, sizeof(ui_static_ui_Ungroup_12));
}

void _ui_remove_static_ui_Ungroup_12() {
    ui_static_ui_Ungroup_12.option.operate_type = 3;

    ui_proc_string_frame(&ui_static_ui_Ungroup_12);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_Ungroup_12, sizeof(ui_static_ui_Ungroup_12));
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
    _ui_init_static_ui_Ungroup_10();
    _ui_init_static_ui_Ungroup_11();
    _ui_init_static_ui_Ungroup_12();
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
    _ui_update_static_ui_Ungroup_10();
    _ui_update_static_ui_Ungroup_11();
    _ui_update_static_ui_Ungroup_12();
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
    _ui_remove_static_ui_Ungroup_10();
    _ui_remove_static_ui_Ungroup_11();
    _ui_remove_static_ui_Ungroup_12();
}

ui_1_frame_t ui_static_ui_group4_0;

ui_interface_arc_t *ui_static_ui_group4_power = (ui_interface_arc_t*)&(ui_static_ui_group4_0.data[0]);

void _ui_init_static_ui_group4_0() {
    for (int i = 0; i < 1; i++) {
        ui_static_ui_group4_0.data[i].figure_name[0] = 2;
        ui_static_ui_group4_0.data[i].figure_name[1] = 1;
        ui_static_ui_group4_0.data[i].figure_name[2] = i + 0;
        ui_static_ui_group4_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_static_ui_group4_0.data[i].operate_type = 0;
    }

    ui_static_ui_group4_power->figure_type = 4;
    ui_static_ui_group4_power->operate_type = 1;
    ui_static_ui_group4_power->layer = 0;
    ui_static_ui_group4_power->color = 6;
    ui_static_ui_group4_power->start_x = 965;
    ui_static_ui_group4_power->start_y = 532;
    ui_static_ui_group4_power->width = 22;
    ui_static_ui_group4_power->start_angle = 232;
    ui_static_ui_group4_power->end_angle = 233;
    ui_static_ui_group4_power->rx = 370;
    ui_static_ui_group4_power->ry = 360;


    ui_proc_1_frame(&ui_static_ui_group4_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_group4_0, sizeof(ui_static_ui_group4_0));
}

void _ui_update_static_ui_group4_0() {
    for (int i = 0; i < 1; i++) {
        ui_static_ui_group4_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_static_ui_group4_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_group4_0, sizeof(ui_static_ui_group4_0));
}

void _ui_remove_static_ui_group4_0() {
    for (int i = 0; i < 1; i++) {
        ui_static_ui_group4_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_static_ui_group4_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_group4_0, sizeof(ui_static_ui_group4_0));
}


void ui_init_static_ui_group4() {
    _ui_init_static_ui_group4_0();
}

void ui_update_static_ui_group4() {
    _ui_update_static_ui_group4_0();
}

void ui_remove_static_ui_group4() {
    _ui_remove_static_ui_group4_0();
}

ui_1_frame_t ui_static_ui_group5_0;

ui_interface_line_t *ui_static_ui_group5_current_angle = (ui_interface_line_t*)&(ui_static_ui_group5_0.data[0]);

void _ui_init_static_ui_group5_0() {
    for (int i = 0; i < 1; i++) {
        ui_static_ui_group5_0.data[i].figure_name[0] = 2;
        ui_static_ui_group5_0.data[i].figure_name[1] = 2;
        ui_static_ui_group5_0.data[i].figure_name[2] = i + 0;
        ui_static_ui_group5_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_static_ui_group5_0.data[i].operate_type = 0;
    }

    ui_static_ui_group5_current_angle->figure_type = 0;
    ui_static_ui_group5_current_angle->operate_type = 1;
    ui_static_ui_group5_current_angle->layer = 0;
    ui_static_ui_group5_current_angle->color = 0;
    ui_static_ui_group5_current_angle->start_x = 1284;
    ui_static_ui_group5_current_angle->start_y = 538;
    ui_static_ui_group5_current_angle->width = 7;
    ui_static_ui_group5_current_angle->end_x = 1305;
    ui_static_ui_group5_current_angle->end_y = 538;


    ui_proc_1_frame(&ui_static_ui_group5_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_group5_0, sizeof(ui_static_ui_group5_0));
}

void _ui_update_static_ui_group5_0() {
    for (int i = 0; i < 1; i++) {
        ui_static_ui_group5_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_static_ui_group5_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_group5_0, sizeof(ui_static_ui_group5_0));
}

void _ui_remove_static_ui_group5_0() {
    for (int i = 0; i < 1; i++) {
        ui_static_ui_group5_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_static_ui_group5_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_group5_0, sizeof(ui_static_ui_group5_0));
}


void ui_init_static_ui_group5() {
    _ui_init_static_ui_group5_0();
}

void ui_update_static_ui_group5() {
    _ui_update_static_ui_group5_0();
}

void ui_remove_static_ui_group5() {
    _ui_remove_static_ui_group5_0();
}

ui_1_frame_t ui_static_ui_group7_0;

ui_interface_line_t *ui_static_ui_group7_gimbal = (ui_interface_line_t*)&(ui_static_ui_group7_0.data[0]);

void _ui_init_static_ui_group7_0() {
    for (int i = 0; i < 1; i++) {
        ui_static_ui_group7_0.data[i].figure_name[0] = 2;
        ui_static_ui_group7_0.data[i].figure_name[1] = 3;
        ui_static_ui_group7_0.data[i].figure_name[2] = i + 0;
        ui_static_ui_group7_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_static_ui_group7_0.data[i].operate_type = 0;
    }

    ui_static_ui_group7_gimbal->figure_type = 0;
    ui_static_ui_group7_gimbal->operate_type = 1;
    ui_static_ui_group7_gimbal->layer = 0;
    ui_static_ui_group7_gimbal->color = 6;
    ui_static_ui_group7_gimbal->start_x = 245;
    ui_static_ui_group7_gimbal->start_y = 620;
    ui_static_ui_group7_gimbal->width = 3;
    ui_static_ui_group7_gimbal->end_x = 245;
    ui_static_ui_group7_gimbal->end_y = 689;


    ui_proc_1_frame(&ui_static_ui_group7_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_group7_0, sizeof(ui_static_ui_group7_0));
}

void _ui_update_static_ui_group7_0() {
    for (int i = 0; i < 1; i++) {
        ui_static_ui_group7_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_static_ui_group7_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_group7_0, sizeof(ui_static_ui_group7_0));
}

void _ui_remove_static_ui_group7_0() {
    for (int i = 0; i < 1; i++) {
        ui_static_ui_group7_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_static_ui_group7_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_group7_0, sizeof(ui_static_ui_group7_0));
}


void ui_init_static_ui_group7() {
    _ui_init_static_ui_group7_0();
}

void ui_update_static_ui_group7() {
    _ui_update_static_ui_group7_0();
}

void ui_remove_static_ui_group7() {
    _ui_remove_static_ui_group7_0();
}


ui_string_frame_t ui_static_ui_group8_0;
ui_interface_string_t* ui_static_ui_group8_FIRE = &(ui_static_ui_group8_0.option);

void _ui_init_static_ui_group8_0() {
    ui_static_ui_group8_0.option.figure_name[0] = 2;
    ui_static_ui_group8_0.option.figure_name[1] = 4;
    ui_static_ui_group8_0.option.figure_name[2] = 0;
    ui_static_ui_group8_0.option.operate_type = 1;

    ui_static_ui_group8_FIRE->figure_type = 7;
    ui_static_ui_group8_FIRE->operate_type = 1;
    ui_static_ui_group8_FIRE->layer = 0;
    ui_static_ui_group8_FIRE->color = 8;
    ui_static_ui_group8_FIRE->start_x = 421;
    ui_static_ui_group8_FIRE->start_y = 653;
    ui_static_ui_group8_FIRE->width = 3;
    ui_static_ui_group8_FIRE->font_size = 28;
    ui_static_ui_group8_FIRE->str_length = 4;
    strcpy(ui_static_ui_group8_FIRE->string, "FIRE");


    ui_proc_string_frame(&ui_static_ui_group8_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_group8_0, sizeof(ui_static_ui_group8_0));
}

void _ui_update_static_ui_group8_0() {
    ui_static_ui_group8_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_static_ui_group8_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_group8_0, sizeof(ui_static_ui_group8_0));
}

void _ui_remove_static_ui_group8_0() {
    ui_static_ui_group8_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_static_ui_group8_0);
    SEND_MESSAGE((uint8_t *) &ui_static_ui_group8_0, sizeof(ui_static_ui_group8_0));
}

void ui_init_static_ui_group8() {
    _ui_init_static_ui_group8_0();
}

void ui_update_static_ui_group8() {
    _ui_update_static_ui_group8_0();
}

void ui_remove_static_ui_group8() {
    _ui_remove_static_ui_group8_0();
}

