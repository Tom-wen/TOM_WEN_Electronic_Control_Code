//
// Created by 21341 on 25-11-18.
//

#ifndef __CONTROL_POWER_H__
#define __CONTROL_POWER_H__

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "chassis_task.h"


void Power_Control(chassis_move_t *chassis_power_control);
void super_power_limit(chassis_move_t *chassis_power_control);

#endif
