#include "control_power.h"
#include "chassis_task.h"
#include "arm_math.h"
#include "referee_usart_task.h"
#include "CAN_receive.h"


 float limit_k=1.0f;


void Power_Control(chassis_move_t *chassis_power_control)
{
    float chassis_power_buffer = referee_data.power_heat.buffer_energy;
	if(chassis_power_buffer<50&&chassis_power_buffer>=40)	limit_k=0.5;
	else if(chassis_power_buffer<40&&chassis_power_buffer>=35)	limit_k=0.5;
	else if(chassis_power_buffer<35&&chassis_power_buffer>=30)	limit_k=0.5;
	else if(chassis_power_buffer<30&&chassis_power_buffer>=20)	limit_k=0.25;
	else if(chassis_power_buffer<20&&chassis_power_buffer>=10)	limit_k=0.125;
	else if(chassis_power_buffer<10&&chassis_power_buffer>=0)	limit_k=0.05;
	else if(chassis_power_buffer>=60)					limit_k=1;
	chassis_power_control->motor_speed_pid[0].out*=limit_k;
    chassis_power_control->motor_speed_pid[1].out*=limit_k;
    chassis_power_control->motor_speed_pid[2].out*=limit_k;
    chassis_power_control->motor_speed_pid[3].out*=limit_k;

}

void super_power_limit(chassis_move_t *chassis_power_control)
{
	static float limit_super_K=1.0f;
	if(supercap_data.capacitor_current > 8.0f || supercap_data.capacitor_voltage < 10.0f)
	{
		limit_super_K=0.8f;
		chassis_power_control->motor_speed_pid[0].out*=limit_super_K;
		chassis_power_control->motor_speed_pid[1].out*=limit_super_K;
		chassis_power_control->motor_speed_pid[2].out*=limit_super_K;
		chassis_power_control->motor_speed_pid[3].out*=limit_super_K;
	}
}

