//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"
#include "CAN_receive.h"

ui_1_frame_t ui_dynamic_ui_group1_0;

ui_interface_rect_t *ui_dynamic_ui_group1_dynamic_speed1 = (ui_interface_rect_t*)&(ui_dynamic_ui_group1_0.data[0]);

void _ui_init_dynamic_ui_group1_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group1_0.data[i].figure_name[0] = 2;
        ui_dynamic_ui_group1_0.data[i].figure_name[1] = 0;
        ui_dynamic_ui_group1_0.data[i].figure_name[2] = i + 0;
        ui_dynamic_ui_group1_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_dynamic_ui_group1_0.data[i].operate_type = 0;
    }

    ui_dynamic_ui_group1_dynamic_speed1->figure_type = 1;
    ui_dynamic_ui_group1_dynamic_speed1->operate_type = 1;
    ui_dynamic_ui_group1_dynamic_speed1->layer = 0;
    ui_dynamic_ui_group1_dynamic_speed1->color = 0;
    ui_dynamic_ui_group1_dynamic_speed1->start_x = 1400;
    ui_dynamic_ui_group1_dynamic_speed1->start_y = 620;
    ui_dynamic_ui_group1_dynamic_speed1->width = 30;
    ui_dynamic_ui_group1_dynamic_speed1->end_x = 1401;
    ui_dynamic_ui_group1_dynamic_speed1->end_y = 621;


    ui_proc_1_frame(&ui_dynamic_ui_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group1_0, sizeof(ui_dynamic_ui_group1_0));
}

void _ui_update_dynamic_ui_group1_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group1_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group1_0, sizeof(ui_dynamic_ui_group1_0));
}

void _ui_remove_dynamic_ui_group1_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group1_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group1_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group1_0, sizeof(ui_dynamic_ui_group1_0));
}


void ui_init_dynamic_ui_group1() {
    _ui_init_dynamic_ui_group1_0();
}

void ui_update_dynamic_ui_group1() {
    _ui_update_dynamic_ui_group1_0();
}

void ui_remove_dynamic_ui_group1() {
    _ui_remove_dynamic_ui_group1_0();
}

ui_1_frame_t ui_dynamic_ui_group2_0;

ui_interface_rect_t *ui_dynamic_ui_group2_Rect_1 = (ui_interface_rect_t*)&(ui_dynamic_ui_group2_0.data[0]);

void _ui_init_dynamic_ui_group2_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group2_0.data[i].figure_name[0] = 2;
        ui_dynamic_ui_group2_0.data[i].figure_name[1] = 1;
        ui_dynamic_ui_group2_0.data[i].figure_name[2] = i + 0;
        ui_dynamic_ui_group2_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_dynamic_ui_group2_0.data[i].operate_type = 0;
    }

    ui_dynamic_ui_group2_Rect_1->figure_type = 1;
    ui_dynamic_ui_group2_Rect_1->operate_type = 1;
    ui_dynamic_ui_group2_Rect_1->layer = 0;
    ui_dynamic_ui_group2_Rect_1->color = 6;
    ui_dynamic_ui_group2_Rect_1->start_x = 60;
    ui_dynamic_ui_group2_Rect_1->start_y = 780;
    ui_dynamic_ui_group2_Rect_1->width = 1;
    ui_dynamic_ui_group2_Rect_1->end_x = 460;
    ui_dynamic_ui_group2_Rect_1->end_y = 825;


    ui_proc_1_frame(&ui_dynamic_ui_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group2_0, sizeof(ui_dynamic_ui_group2_0));
}

void _ui_update_dynamic_ui_group2_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group2_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group2_0, sizeof(ui_dynamic_ui_group2_0));
}

void _ui_remove_dynamic_ui_group2_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group2_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group2_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group2_0, sizeof(ui_dynamic_ui_group2_0));
}


void ui_init_dynamic_ui_group2() {
    _ui_init_dynamic_ui_group2_0();
}

void ui_update_dynamic_ui_group2() {
    _ui_update_dynamic_ui_group2_0();
}

void ui_remove_dynamic_ui_group2() {
    _ui_remove_dynamic_ui_group2_0();
}

ui_1_frame_t ui_dynamic_ui_group3_0;

ui_interface_rect_t *ui_dynamic_ui_group3_Rect_2 = (ui_interface_rect_t*)&(ui_dynamic_ui_group3_0.data[0]);

void _ui_init_dynamic_ui_group3_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group3_0.data[i].figure_name[0] = 2;
        ui_dynamic_ui_group3_0.data[i].figure_name[1] = 2;
        ui_dynamic_ui_group3_0.data[i].figure_name[2] = i + 0;
        ui_dynamic_ui_group3_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_dynamic_ui_group3_0.data[i].operate_type = 0;
    }

    ui_dynamic_ui_group3_Rect_2->figure_type = 1;
    ui_dynamic_ui_group3_Rect_2->operate_type = 1;
    ui_dynamic_ui_group3_Rect_2->layer = 0;
    ui_dynamic_ui_group3_Rect_2->color = 6;
    ui_dynamic_ui_group3_Rect_2->start_x = 60;
    ui_dynamic_ui_group3_Rect_2->start_y = 735;
    ui_dynamic_ui_group3_Rect_2->width = 1;
    ui_dynamic_ui_group3_Rect_2->end_x = 460;
    ui_dynamic_ui_group3_Rect_2->end_y = 780;


    ui_proc_1_frame(&ui_dynamic_ui_group3_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group3_0, sizeof(ui_dynamic_ui_group3_0));
}

void _ui_update_dynamic_ui_group3_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group3_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group3_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group3_0, sizeof(ui_dynamic_ui_group3_0));
}

void _ui_remove_dynamic_ui_group3_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group3_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group3_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group3_0, sizeof(ui_dynamic_ui_group3_0));
}


void ui_init_dynamic_ui_group3() {
    _ui_init_dynamic_ui_group3_0();
}

void ui_update_dynamic_ui_group3() {
    _ui_update_dynamic_ui_group3_0();
}

void ui_remove_dynamic_ui_group3() {
    _ui_remove_dynamic_ui_group3_0();
}

ui_1_frame_t ui_dynamic_ui_group4_0;

ui_interface_rect_t *ui_dynamic_ui_group4_Rect_3 = (ui_interface_rect_t*)&(ui_dynamic_ui_group4_0.data[0]);

void _ui_init_dynamic_ui_group4_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group4_0.data[i].figure_name[0] = 2;
        ui_dynamic_ui_group4_0.data[i].figure_name[1] = 3;
        ui_dynamic_ui_group4_0.data[i].figure_name[2] = i + 0;
        ui_dynamic_ui_group4_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_dynamic_ui_group4_0.data[i].operate_type = 0;
    }

    ui_dynamic_ui_group4_Rect_3->figure_type = 1;
    ui_dynamic_ui_group4_Rect_3->operate_type = 1;
    ui_dynamic_ui_group4_Rect_3->layer = 0;
    ui_dynamic_ui_group4_Rect_3->color = 6;
    ui_dynamic_ui_group4_Rect_3->start_x = 60;
    ui_dynamic_ui_group4_Rect_3->start_y = 690;
    ui_dynamic_ui_group4_Rect_3->width = 1;
    ui_dynamic_ui_group4_Rect_3->end_x = 460;
    ui_dynamic_ui_group4_Rect_3->end_y = 735;


    ui_proc_1_frame(&ui_dynamic_ui_group4_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group4_0, sizeof(ui_dynamic_ui_group4_0));
}

void _ui_update_dynamic_ui_group4_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group4_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group4_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group4_0, sizeof(ui_dynamic_ui_group4_0));
}

void _ui_remove_dynamic_ui_group4_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group4_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group4_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group4_0, sizeof(ui_dynamic_ui_group4_0));
}


void ui_init_dynamic_ui_group4() {
    _ui_init_dynamic_ui_group4_0();
}

void ui_update_dynamic_ui_group4() {
    _ui_update_dynamic_ui_group4_0();
}

void ui_remove_dynamic_ui_group4() {
    _ui_remove_dynamic_ui_group4_0();
}

ui_1_frame_t ui_dynamic_ui_group5_0;

ui_interface_rect_t *ui_dynamic_ui_group5_Rect_4 = (ui_interface_rect_t*)&(ui_dynamic_ui_group5_0.data[0]);

void _ui_init_dynamic_ui_group5_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group5_0.data[i].figure_name[0] = 2;
        ui_dynamic_ui_group5_0.data[i].figure_name[1] = 4;
        ui_dynamic_ui_group5_0.data[i].figure_name[2] = i + 0;
        ui_dynamic_ui_group5_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_dynamic_ui_group5_0.data[i].operate_type = 0;
    }

    ui_dynamic_ui_group5_Rect_4->figure_type = 1;
    ui_dynamic_ui_group5_Rect_4->operate_type = 1;
    ui_dynamic_ui_group5_Rect_4->layer = 0;
    ui_dynamic_ui_group5_Rect_4->color = 6;
    ui_dynamic_ui_group5_Rect_4->start_x = 60;
    ui_dynamic_ui_group5_Rect_4->start_y = 570;
    ui_dynamic_ui_group5_Rect_4->width = 1;
    ui_dynamic_ui_group5_Rect_4->end_x = 460;
    ui_dynamic_ui_group5_Rect_4->end_y = 615;


    ui_proc_1_frame(&ui_dynamic_ui_group5_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group5_0, sizeof(ui_dynamic_ui_group5_0));
}

void _ui_update_dynamic_ui_group5_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group5_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group5_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group5_0, sizeof(ui_dynamic_ui_group5_0));
}

void _ui_remove_dynamic_ui_group5_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group5_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group5_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group5_0, sizeof(ui_dynamic_ui_group5_0));
}


void ui_init_dynamic_ui_group5() {
    _ui_init_dynamic_ui_group5_0();
}

void ui_update_dynamic_ui_group5() {
    _ui_update_dynamic_ui_group5_0();
}

void ui_remove_dynamic_ui_group5() {
    _ui_remove_dynamic_ui_group5_0();
}

ui_1_frame_t ui_dynamic_ui_group6_0;

ui_interface_rect_t *ui_dynamic_ui_group6_Rect_5 = (ui_interface_rect_t*)&(ui_dynamic_ui_group6_0.data[0]);

void _ui_init_dynamic_ui_group6_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group6_0.data[i].figure_name[0] = 2;
        ui_dynamic_ui_group6_0.data[i].figure_name[1] = 5;
        ui_dynamic_ui_group6_0.data[i].figure_name[2] = i + 0;
        ui_dynamic_ui_group6_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_dynamic_ui_group6_0.data[i].operate_type = 0;
    }

    ui_dynamic_ui_group6_Rect_5->figure_type = 1;
    ui_dynamic_ui_group6_Rect_5->operate_type = 1;
    ui_dynamic_ui_group6_Rect_5->layer = 0;
    ui_dynamic_ui_group6_Rect_5->color = 6;
    ui_dynamic_ui_group6_Rect_5->start_x = 60;
    ui_dynamic_ui_group6_Rect_5->start_y = 525;
    ui_dynamic_ui_group6_Rect_5->width = 1;
    ui_dynamic_ui_group6_Rect_5->end_x = 460;
    ui_dynamic_ui_group6_Rect_5->end_y = 570;


    ui_proc_1_frame(&ui_dynamic_ui_group6_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group6_0, sizeof(ui_dynamic_ui_group6_0));
}

void _ui_update_dynamic_ui_group6_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group6_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group6_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group6_0, sizeof(ui_dynamic_ui_group6_0));
}

void _ui_remove_dynamic_ui_group6_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group6_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group6_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group6_0, sizeof(ui_dynamic_ui_group6_0));
}


void ui_init_dynamic_ui_group6() {
    _ui_init_dynamic_ui_group6_0();
}

void ui_update_dynamic_ui_group6() {
    _ui_update_dynamic_ui_group6_0();
}

void ui_remove_dynamic_ui_group6() {
    _ui_remove_dynamic_ui_group6_0();
}

ui_1_frame_t ui_dynamic_ui_group7_0;

ui_interface_rect_t *ui_dynamic_ui_group7_Rect_6 = (ui_interface_rect_t*)&(ui_dynamic_ui_group7_0.data[0]);

void _ui_init_dynamic_ui_group7_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group7_0.data[i].figure_name[0] = 2;
        ui_dynamic_ui_group7_0.data[i].figure_name[1] = 6;
        ui_dynamic_ui_group7_0.data[i].figure_name[2] = i + 0;
        ui_dynamic_ui_group7_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_dynamic_ui_group7_0.data[i].operate_type = 0;
    }

    ui_dynamic_ui_group7_Rect_6->figure_type = 1;
    ui_dynamic_ui_group7_Rect_6->operate_type = 1;
    ui_dynamic_ui_group7_Rect_6->layer = 0;
    ui_dynamic_ui_group7_Rect_6->color = 0;
    ui_dynamic_ui_group7_Rect_6->start_x = 60;
    ui_dynamic_ui_group7_Rect_6->start_y = 480;
    ui_dynamic_ui_group7_Rect_6->width = 1;
    ui_dynamic_ui_group7_Rect_6->end_x = 460;
    ui_dynamic_ui_group7_Rect_6->end_y = 525;


    ui_proc_1_frame(&ui_dynamic_ui_group7_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group7_0, sizeof(ui_dynamic_ui_group7_0));
}

void _ui_update_dynamic_ui_group7_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group7_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group7_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group7_0, sizeof(ui_dynamic_ui_group7_0));
}

void _ui_remove_dynamic_ui_group7_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group7_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group7_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group7_0, sizeof(ui_dynamic_ui_group7_0));
}


void ui_init_dynamic_ui_group7() {
    _ui_init_dynamic_ui_group7_0();
}

void ui_update_dynamic_ui_group7() {
    _ui_update_dynamic_ui_group7_0();
}

void ui_remove_dynamic_ui_group7() {
    _ui_remove_dynamic_ui_group7_0();
}

ui_1_frame_t ui_dynamic_ui_group8_0;

ui_interface_number_t *ui_dynamic_ui_group8_super_cap = (ui_interface_number_t*)&(ui_dynamic_ui_group8_0.data[0]);

void _ui_init_dynamic_ui_group8_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group8_0.data[i].figure_name[0] = 1;
        ui_dynamic_ui_group8_0.data[i].figure_name[1] = 7;
        ui_dynamic_ui_group8_0.data[i].figure_name[2] = i + 0;
        ui_dynamic_ui_group8_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_dynamic_ui_group8_0.data[i].operate_type = 0;
    }

    ui_dynamic_ui_group8_super_cap->figure_type = 5;
    ui_dynamic_ui_group8_super_cap->operate_type = 1;
    ui_dynamic_ui_group8_super_cap->layer = 0;
    ui_dynamic_ui_group8_super_cap->color = 6;
    ui_dynamic_ui_group8_super_cap->start_x = 1480;
    ui_dynamic_ui_group8_super_cap->start_y = 725;
    ui_dynamic_ui_group8_super_cap->width = 2;
    ui_dynamic_ui_group8_super_cap->font_size = 20;
    ui_dynamic_ui_group8_super_cap->number = (uint16_t)(supercap_data.capacitor_level*10);


    ui_proc_1_frame(&ui_dynamic_ui_group8_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group8_0, sizeof(ui_dynamic_ui_group8_0));
}

void _ui_update_dynamic_ui_group8_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group8_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group8_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group8_0, sizeof(ui_dynamic_ui_group8_0));
}

void _ui_remove_dynamic_ui_group8_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group8_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group8_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group8_0, sizeof(ui_dynamic_ui_group8_0));
}


void ui_init_dynamic_ui_group8() {
    _ui_init_dynamic_ui_group8_0();
}

void ui_update_dynamic_ui_group8() {
    _ui_update_dynamic_ui_group8_0();
}

void ui_remove_dynamic_ui_group8() {
    _ui_remove_dynamic_ui_group8_0();
}


ui_1_frame_t ui_dynamic_ui_group9_0;

ui_interface_arc_t *ui_dynamic_ui_group9_power = (ui_interface_arc_t*)&(ui_dynamic_ui_group9_0.data[0]);

void _ui_init_dynamic_ui_group9_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group9_0.data[i].figure_name[0] = 2;
        ui_dynamic_ui_group9_0.data[i].figure_name[1] = 8;
        ui_dynamic_ui_group9_0.data[i].figure_name[2] = i + 0;
        ui_dynamic_ui_group9_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_dynamic_ui_group9_0.data[i].operate_type = 0;
    }

    ui_dynamic_ui_group9_power->figure_type = 4;
    ui_dynamic_ui_group9_power->operate_type = 1;
    ui_dynamic_ui_group9_power->layer = 0;
    ui_dynamic_ui_group9_power->color = 6;
    ui_dynamic_ui_group9_power->start_x = 965;
    ui_dynamic_ui_group9_power->start_y = 532;
    ui_dynamic_ui_group9_power->width = 22;
    ui_dynamic_ui_group9_power->start_angle = 232;
    ui_dynamic_ui_group9_power->end_angle = 233;
    ui_dynamic_ui_group9_power->rx = 370;
    ui_dynamic_ui_group9_power->ry = 360;


    ui_proc_1_frame(&ui_dynamic_ui_group9_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group9_0, sizeof(ui_dynamic_ui_group9_0));
}

void _ui_update_dynamic_ui_group9_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group9_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group9_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group9_0, sizeof(ui_dynamic_ui_group9_0));
}

void _ui_remove_dynamic_ui_group9_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group9_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group9_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group9_0, sizeof(ui_dynamic_ui_group9_0));
}


void ui_init_dynamic_ui_group9() {
    _ui_init_dynamic_ui_group9_0();
}

void ui_update_dynamic_ui_group9() {
    _ui_update_dynamic_ui_group9_0();
}

void ui_remove_dynamic_ui_group9() {
    _ui_remove_dynamic_ui_group9_0();
}


ui_1_frame_t ui_dynamic_ui_group10_0;

ui_interface_line_t *ui_dynamic_ui_group10_current_angle = (ui_interface_line_t*)&(ui_dynamic_ui_group10_0.data[0]);

void _ui_init_dynamic_ui_group10_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group10_0.data[i].figure_name[0] = 2;
        ui_dynamic_ui_group10_0.data[i].figure_name[1] = 9;
        ui_dynamic_ui_group10_0.data[i].figure_name[2] = i + 0;
        ui_dynamic_ui_group10_0.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_dynamic_ui_group10_0.data[i].operate_type = 0;
    }

    ui_dynamic_ui_group10_current_angle->figure_type = 0;
    ui_dynamic_ui_group10_current_angle->operate_type = 1;
    ui_dynamic_ui_group10_current_angle->layer = 0;
    ui_dynamic_ui_group10_current_angle->color = 0;
    ui_dynamic_ui_group10_current_angle->start_x = 1284;
    ui_dynamic_ui_group10_current_angle->start_y = 538;
    ui_dynamic_ui_group10_current_angle->width = 7;
    ui_dynamic_ui_group10_current_angle->end_x = 1305;
    ui_dynamic_ui_group10_current_angle->end_y = 538;


    ui_proc_1_frame(&ui_dynamic_ui_group10_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group10_0, sizeof(ui_dynamic_ui_group10_0));
}

void _ui_update_dynamic_ui_group10_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group10_0.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group10_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group10_0, sizeof(ui_dynamic_ui_group10_0));
}

void _ui_remove_dynamic_ui_group10_0() {
    for (int i = 0; i < 1; i++) {
        ui_dynamic_ui_group10_0.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_dynamic_ui_group10_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group10_0, sizeof(ui_dynamic_ui_group10_0));
}


void ui_init_dynamic_ui_group10() {
    _ui_init_dynamic_ui_group10_0();
}

void ui_update_dynamic_ui_group10() {
    _ui_update_dynamic_ui_group10_0();
}

void ui_remove_dynamic_ui_group10() {
    _ui_remove_dynamic_ui_group10_0();
}



ui_string_frame_t ui_dynamic_ui_group11_0;
ui_interface_string_t* ui_dynamic_ui_group11_high_speed_TOP_mode = &(ui_dynamic_ui_group11_0.option);

void _ui_init_dynamic_ui_group11_0() {
    ui_dynamic_ui_group11_0.option.figure_name[0] = 1;
    ui_dynamic_ui_group11_0.option.figure_name[1] = 10;
    ui_dynamic_ui_group11_0.option.figure_name[2] = 0;
    ui_dynamic_ui_group11_0.option.operate_type = 1;

    ui_dynamic_ui_group11_high_speed_TOP_mode->figure_type = 7;
    ui_dynamic_ui_group11_high_speed_TOP_mode->operate_type = 1;
    ui_dynamic_ui_group11_high_speed_TOP_mode->layer = 0;
    ui_dynamic_ui_group11_high_speed_TOP_mode->color = 8;
    ui_dynamic_ui_group11_high_speed_TOP_mode->start_x = 754;
    ui_dynamic_ui_group11_high_speed_TOP_mode->start_y = 187;
    ui_dynamic_ui_group11_high_speed_TOP_mode->width = 2;
    ui_dynamic_ui_group11_high_speed_TOP_mode->font_size = 20;
    ui_dynamic_ui_group11_high_speed_TOP_mode->str_length = 21;
    strcpy(ui_dynamic_ui_group11_high_speed_TOP_mode->string, "HIGHT_SPEED_TOP");


    ui_proc_string_frame(&ui_dynamic_ui_group11_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group11_0, sizeof(ui_dynamic_ui_group11_0));
}

void _ui_update_dynamic_ui_group11_0() {
    ui_dynamic_ui_group11_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_dynamic_ui_group11_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group11_0, sizeof(ui_dynamic_ui_group11_0));
}

void _ui_remove_dynamic_ui_group11_0() {
    ui_dynamic_ui_group11_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_dynamic_ui_group11_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group11_0, sizeof(ui_dynamic_ui_group11_0));
}

void ui_init_dynamic_ui_group11() {
    _ui_init_dynamic_ui_group11_0();
}

void ui_update_dynamic_ui_group11() {
    _ui_update_dynamic_ui_group11_0();
}

void ui_remove_dynamic_ui_group11() {
    _ui_remove_dynamic_ui_group11_0();
}


ui_string_frame_t ui_dynamic_ui_group12_0;
ui_interface_string_t* ui_dynamic_ui_group12_low_speed_TOP_mode = &(ui_dynamic_ui_group12_0.option);

void _ui_init_dynamic_ui_group12_0() {
    ui_dynamic_ui_group12_0.option.figure_name[0] = 1;
    ui_dynamic_ui_group12_0.option.figure_name[1] = 11;
    ui_dynamic_ui_group12_0.option.figure_name[2] = 1;
    ui_dynamic_ui_group12_0.option.operate_type = 1;

    ui_dynamic_ui_group12_low_speed_TOP_mode->figure_type = 7;
    ui_dynamic_ui_group12_low_speed_TOP_mode->operate_type = 1;
    ui_dynamic_ui_group12_low_speed_TOP_mode->layer = 0;
    ui_dynamic_ui_group12_low_speed_TOP_mode->color = 8;
    ui_dynamic_ui_group12_low_speed_TOP_mode->start_x = 811;
    ui_dynamic_ui_group12_low_speed_TOP_mode->start_y = 138;
    ui_dynamic_ui_group12_low_speed_TOP_mode->width = 2;
    ui_dynamic_ui_group12_low_speed_TOP_mode->font_size = 20;
    ui_dynamic_ui_group12_low_speed_TOP_mode->str_length = 14;
    strcpy(ui_dynamic_ui_group12_low_speed_TOP_mode->string, "LOW_SPEED_TOP");


    ui_proc_string_frame(&ui_dynamic_ui_group12_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group12_0, sizeof(ui_dynamic_ui_group12_0));
}

void _ui_update_dynamic_ui_group12_0() {
    ui_dynamic_ui_group12_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_dynamic_ui_group12_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group12_0, sizeof(ui_dynamic_ui_group12_0));
}

void _ui_remove_dynamic_ui_group12_0() {
    ui_dynamic_ui_group12_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_dynamic_ui_group12_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group12_0, sizeof(ui_dynamic_ui_group12_0));
}

void ui_init_dynamic_ui_group12() {
    _ui_init_dynamic_ui_group12_0();
}

void ui_update_dynamic_ui_group12() {
    _ui_update_dynamic_ui_group12_0();
}

void ui_remove_dynamic_ui_group12() {
    _ui_remove_dynamic_ui_group12_0();
}


ui_string_frame_t ui_dynamic_ui_group13_0;
ui_interface_string_t* ui_dynamic_ui_group13_normal_speed_mode = &(ui_dynamic_ui_group13_0.option);


void _ui_init_dynamic_ui_group13_0() {
    ui_dynamic_ui_group13_0.option.figure_name[0] = 1;
    ui_dynamic_ui_group13_0.option.figure_name[1] = 12;
    ui_dynamic_ui_group13_0.option.figure_name[2] = 1;
    ui_dynamic_ui_group13_0.option.operate_type = 1;

    ui_dynamic_ui_group13_normal_speed_mode->figure_type = 7;
    ui_dynamic_ui_group13_normal_speed_mode->operate_type = 1;
    ui_dynamic_ui_group13_normal_speed_mode->layer = 0;
    ui_dynamic_ui_group13_normal_speed_mode->color = 8;
    ui_dynamic_ui_group13_normal_speed_mode->start_x = 1500;
    ui_dynamic_ui_group13_normal_speed_mode->start_y = 640;
    ui_dynamic_ui_group13_normal_speed_mode->width = 2;
    ui_dynamic_ui_group13_normal_speed_mode->font_size = 20;
    ui_dynamic_ui_group13_normal_speed_mode->str_length = 14;
    strcpy(ui_dynamic_ui_group13_normal_speed_mode->string, "NORMAL_SPEED");


    ui_proc_string_frame(&ui_dynamic_ui_group13_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group13_0, sizeof(ui_dynamic_ui_group13_0));
}

void _ui_update_dynamic_ui_group13_0() {
    ui_dynamic_ui_group13_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_dynamic_ui_group13_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group13_0, sizeof(ui_dynamic_ui_group13_0));
}

void _ui_remove_dynamic_ui_group13_0() {
    ui_dynamic_ui_group13_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_dynamic_ui_group13_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group13_0, sizeof(ui_dynamic_ui_group13_0));
}

void ui_init_dynamic_ui_group13() {
    _ui_init_dynamic_ui_group13_0();
}

void ui_update_dynamic_ui_group13() {
    _ui_update_dynamic_ui_group13_0();
}

void ui_remove_dynamic_ui_group13() {
    _ui_remove_dynamic_ui_group13_0();
}

ui_string_frame_t ui_dynamic_ui_group14_0;
ui_interface_string_t* ui_dynamic_ui_group14_low_speed_mode = &(ui_dynamic_ui_group14_0.option);


void _ui_init_dynamic_ui_group14_0() {
    ui_dynamic_ui_group14_0.option.figure_name[0] = 1;
    ui_dynamic_ui_group14_0.option.figure_name[1] = 13;
    ui_dynamic_ui_group14_0.option.figure_name[2] = 1;
    ui_dynamic_ui_group14_0.option.operate_type = 1;

    ui_dynamic_ui_group14_low_speed_mode->figure_type = 7;
    ui_dynamic_ui_group14_low_speed_mode->operate_type = 1;
    ui_dynamic_ui_group14_low_speed_mode->layer = 0;
    ui_dynamic_ui_group14_low_speed_mode->color = 8;
    ui_dynamic_ui_group14_low_speed_mode->start_x = 1500;
    ui_dynamic_ui_group14_low_speed_mode->start_y = 600;
    ui_dynamic_ui_group14_low_speed_mode->width = 2;
    ui_dynamic_ui_group14_low_speed_mode->font_size = 20;
    ui_dynamic_ui_group14_low_speed_mode->str_length = 14;
    strcpy(ui_dynamic_ui_group14_low_speed_mode->string, "LOW_SPEED");


    ui_proc_string_frame(&ui_dynamic_ui_group14_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group14_0, sizeof(ui_dynamic_ui_group14_0));
}

void _ui_update_dynamic_ui_group14_0() {
    ui_dynamic_ui_group14_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_dynamic_ui_group14_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group14_0, sizeof(ui_dynamic_ui_group14_0));
}

void _ui_remove_dynamic_ui_group14_0() {
    ui_dynamic_ui_group14_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_dynamic_ui_group14_0);
    SEND_MESSAGE((uint8_t *) &ui_dynamic_ui_group14_0, sizeof(ui_dynamic_ui_group14_0));
}

void ui_init_dynamic_ui_group14() {
    _ui_init_dynamic_ui_group14_0();
}

void ui_update_dynamic_ui_group14() {
    _ui_update_dynamic_ui_group14_0();
}

void ui_remove_dynamic_ui_group14() {
    _ui_remove_dynamic_ui_group14_0();
}