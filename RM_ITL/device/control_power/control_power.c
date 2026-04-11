#include "control_power.h"
#include "Lowpass.h"

pm_power power_receive_data;
super_cap_data super_cap;
MotorPower chassis_3508_motor[4];
MotorPower chassis_6020_motor[4];

static float limit_rate = 1.0f;
static float actual_total_3508_power = 0.0f;
static float actual_total_6020_power = 0.0f;
float chassis_total_power = 0.0f;

void motor_power_init(MotorPower* motor, const motor_power_init_t* init_data) {
    motor->k0 = init_data->k0;
    motor->k1 = init_data->k1;
    motor->k2 = init_data->k2;
    motor->k3 = init_data->k3;
    motor->k4 = init_data->k4;
    motor->k5 = init_data->k5;
    motor->Current_Conversion = init_data->real_current_conversion;
    
    // 初始化其他成员变量为默认值
    motor->power_limit = 0.0;
    motor->feedback_power = 0.0;
    motor->predict_power = 0.0;
    motor->predict_not_limit_power = 0.0;
}

// 创建初始化数据
motor_power_init_t motor3508_init_data[4] = {
    {//步兵左前
        .k0 = 0.65213,
        .k1 = -0.15659,
        .k2 = 0.00041660,
        .k3 = 0.00235415,
        .k4 = 0.20022,
        .k5 = 1.08e-7,
        .real_current_conversion = 819.2
    },
    {//步兵右前
        .k0 = 0.65213,
        .k1 = -0.15659,
        .k2 = 0.00041660,
        .k3 = 0.00235415,
        .k4 = 0.20022,
        .k5 = 1.08e-7,
        .real_current_conversion = 819.2
    },
    {//步兵右后
        .k0 = 0.65213,
        .k1 = -0.15659,
        .k2 = 0.00041660,
        .k3 = 0.00235415,
        .k4 = 0.20022,
        .k5 = 1.08e-7,
        .real_current_conversion = 819.2
    },
    {//步兵左后
        .k0 = 0.65213,
        .k1 = -0.15659,
        .k2 = 0.00041660,
        .k3 = 0.00235415,
        .k4 = 0.20022,
        .k5 = 1.08e-7,
        .real_current_conversion = 819.2
    },
};


// motor_power_init_t motor6020_init_data[4] = {
//     {//步兵左前
//         .k0 = -0.011101188773801894,
//         .k1 = 23.91595460113854,
//         .k2 = 0.00012305478495736283,
//         .k3 = 0.00027388660150835055,
//         .k4 = -0.21141856679155993,
//     },
// };

void Chassis_Power_Init(void){
    for (int i = 0; i < 4; i++){
        motor_power_init(&chassis_3508_motor[i], &motor3508_init_data[i]);
    }
}

float Chassis_3508_Get_Limited_Number(uint8_t motor_id) {
    if (motor_id >= 4) return 0.0f;
    return chassis_3508_motor[motor_id].decay_number;
}

float Chassis_6020_Get_Limited_Number(uint8_t motor_id) {
    if (motor_id >= 4) return 0.0f;
    return chassis_6020_motor[motor_id].decay_number;
}


// 获取电机真实电流的辅助函数
float motor_power_get_real_current(MotorPower *motor, uint8_t motor_id) {
    float real_current = motor[motor_id].current_current / motor[motor_id].Current_Conversion;
    return real_current;
}

//计算底盘3508实际总功率
float Chassis_3508_Actual_Power_Calculate(MotorPower *chassis_3508_motor, uint8_t motor3508_number){
    float total_3508_power = 0;
    for (int i = 0; i < motor3508_number; i++){
        total_3508_power += motor_power_clac(chassis_3508_motor, i);
    }
    return total_3508_power;
}

//计算底盘6020实际总功率
float Chassis_6020_Actual_Power_Calculate(MotorPower *chassis_6020_motor, uint8_t motor6020_number){ 
    float total_6020_power = 0;
    for (int i = 0; i < motor6020_number; i++){
        total_6020_power += motor_power_clac(chassis_6020_motor, i);
    }
    return total_6020_power;
}
// 功率控制
float Power_Control(FDCAN_HandleTypeDef *hfdcan, float total_power_limit, uint8_t motor3508_number, uint8_t motor6020_number)
{
    // 安全参数检查
    if(motor3508_number <= 0 && motor6020_number <= 0)
    {
        return 1.0f;
    }
    
    // 限制电机数量范围
    if(motor3508_number > 4) motor3508_number = 4;
    if(motor6020_number > 4) motor6020_number = 4;

    float need_power_limit[2] = {0};
    float actual_total_6020_power = 0.0f;
    float total_power = 0.0f;
    float allocate_3508_rate = 0.5f;  // 默认均分
    float allocate_6020_rate = 0.5f;

    //向超电发送限制功率
    super_cap_limitpower_sent(hfdcan, total_power_limit);

    // 计算两种电机的总功率
    actual_total_3508_power = Chassis_3508_Actual_Power_Calculate(chassis_3508_motor, motor3508_number);
    actual_total_6020_power = Chassis_6020_Actual_Power_Calculate(chassis_6020_motor, motor6020_number);
    chassis_total_power = actual_total_3508_power + actual_total_6020_power;
    // 安全计算分配比例（防止除零）
    total_power = actual_total_3508_power + actual_total_6020_power;
    if(total_power > 0.001f)
    {
        allocate_3508_rate = actual_total_3508_power / total_power;
        allocate_6020_rate = actual_total_6020_power / total_power;
    }
    else
    {
        // 功率接近0时，按电机数量比例分配
        uint8_t total_motors = motor3508_number + motor6020_number;
        if(total_motors > 0)
        {
            allocate_3508_rate = (float)motor3508_number / total_motors;
            allocate_6020_rate = (float)motor6020_number / total_motors;
        }
    }
    
    // 计算实际限制功率（应用预留百分比）

    float effective_power_limit = total_power_limit * POWER_RESERVE_RATE;
    need_power_limit[motor3508] = effective_power_limit  * allocate_3508_rate;
    need_power_limit[motor6020] = effective_power_limit  * allocate_6020_rate;

    // 当前单个电机实际功率
    float motor3508_current_power[4] = {0};
    float motor6020_current_power[4] = {0};
    // 当前每个电机分配系数
    float motor3508_power_rate[4] = {0};
    float motor6020_power_rate[4] = {0};

    // 计算3508电机功率和分配系数
    for(int i = 0; i < motor3508_number; i++)
    {
        motor3508_current_power[i] = motor_power_clac(chassis_3508_motor, i);
        motor3508_current_power[i] = LowPassFilter_operator(&LPF_current, motor3508_current_power[i]);
        // 安全计算功率比例（防止除零）
        if(actual_total_3508_power > 0.001f)
        {
            motor3508_power_rate[i] = motor3508_current_power[i] / actual_total_3508_power;
        }
        else
        {
            motor3508_power_rate[i] = 1.0f / motor3508_number;  // 均分
        }
    }

    // 计算6020电机功率和分配系数
    for(int i = 0; i < motor6020_number; i++)
    {
        motor6020_current_power[i] = motor_power_clac(chassis_6020_motor, i);
        motor6020_current_power[i] = LowPassFilter_operator(&LPF_current, motor6020_current_power[i]);
        // 安全计算功率比例（防止除零）
        if(actual_total_6020_power > 0.001f)
        {
            motor6020_power_rate[i] = motor6020_current_power[i] / actual_total_6020_power;
        }
        else
        {
            motor6020_power_rate[i] = 1.0f / motor6020_number;  // 均分
        }
    }

    // 计算当前单个电机目标功率
    for(int i = 0; i < motor3508_number; i++)
    {
        chassis_3508_motor[i].power_limit = need_power_limit[motor3508] * motor3508_power_rate[i];
    }
    for(int i = 0; i < motor6020_number; i++)
    {
        chassis_6020_motor[i].power_limit = need_power_limit[motor6020] * motor6020_power_rate[i];
    }

    // 功率超限处理
    if((actual_total_3508_power + actual_total_6020_power) > effective_power_limit)
    {
        float total_speed_error_3508 = 0.0f;
        float total_speed_error_6020 = 0.0f;

        // 计算总功率用于计算整体衰减系数
        float total_actual_power = actual_total_3508_power + actual_total_6020_power;
        
        // 处理3508电机
        // 先计算总速度误差
        for(int i = 0; i < motor3508_number; i++)
        {
            chassis_3508_motor[i].speed_error = fabsf(chassis_3508_motor[i].current_speed - chassis_3508_motor[i].target_speed);
            total_speed_error_3508 += chassis_3508_motor[i].speed_error;
        }
        // 再计算衰减系数
        if(total_speed_error_3508 > 0.001f)
        {
            for(int i = 0; i < motor3508_number; i++)
            {
                chassis_3508_motor[i].ratio = chassis_3508_motor[i].speed_error / total_speed_error_3508;
                chassis_3508_motor[i].predict_power = chassis_3508_motor[i].ratio * need_power_limit[motor3508];
                
                // 安全计算衰减系数（防止除零，限制范围）
                float denominator = (total_actual_power >= POWER_HIGH_THRESHOLD) ? 
                                    (motor3508_current_power[i] + MOTOR_STATIC_POWER) : 
                                    (motor3508_current_power[i] + 0.1f);
                float decay = chassis_3508_motor[i].predict_power / denominator;
                // 限制衰减系数范围 [0.1, 1.5]
                chassis_3508_motor[i].decay_number = fmaxf(0.1f, fminf(decay, 1.5f));
            }
        }
        else
        {
            // 速度误差为0时，默认衰减系数
            for(int i = 0; i < motor3508_number; i++)
            {
                chassis_3508_motor[i].decay_number = 1.0f;
            }
        }

        // 处理6020电机
        // 先计算总速度误差
        for(int i = 0; i < motor6020_number; i++)
        {
            chassis_6020_motor[i].speed_error = fabsf(chassis_6020_motor[i].current_speed - chassis_6020_motor[i].target_speed);
            total_speed_error_6020 += chassis_6020_motor[i].speed_error;
        }
        // 再计算衰减系数
        if(total_speed_error_6020 > 0.001f)
        {
            for(int i = 0; i < motor6020_number; i++)
            {
                chassis_6020_motor[i].ratio = chassis_6020_motor[i].speed_error / total_speed_error_6020;
                chassis_6020_motor[i].predict_power = chassis_6020_motor[i].ratio * need_power_limit[motor6020];
                
                // 安全计算衰减系数（防止除零，限制范围）
                float denominator = (total_actual_power >= POWER_HIGH_THRESHOLD) ? 
                                    (motor6020_current_power[i] + MOTOR_STATIC_POWER) : 
                                    (motor6020_current_power[i] + 0.1f);
                float decay = chassis_6020_motor[i].predict_power / denominator;
                // 限制衰减系数范围 [0.1, 1.5]
                chassis_6020_motor[i].decay_number = fmaxf(0.1f, fminf(decay, 1.5f));
            }
        }
        else
        {
            // 速度误差为0时，默认衰减系数
            for(int i = 0; i < motor6020_number; i++)
            {
                chassis_6020_motor[i].decay_number = 1.0f;
            }
        }

        // 修复：正确计算整体衰减系数作为返回值
        limit_rate = (effective_power_limit ) / total_actual_power;
        limit_rate = fmaxf(0.1f, fminf(limit_rate, 1.0f));
    }
    else
    {
        // 功率未超限，不衰减
        limit_rate = 1.0f;
        for(int i = 0; i < motor3508_number; i++)
        {
            chassis_3508_motor[i].decay_number = limit_rate;
        }
        for(int i = 0; i < motor6020_number; i++)
        {
            chassis_6020_motor[i].decay_number = limit_rate;
        }
    }

    return limit_rate;
}

//计算单个电机功率
float motor_power_clac(MotorPower *motor, uint8_t motor_id)
{
    float motor_power = 0;
    motor[motor_id].cal_current = fabs(motor_power_get_real_current(motor, motor_id));
    motor[motor_id].cal_speed = fabs(motor[motor_id].current_speed);
    motor_power = 1.996e-6 * motor[motor_id].cal_current * motor[motor_id].cal_speed 
    + 1.203e-06 * motor[motor_id].cal_speed * motor[motor_id].cal_speed
    + 5.462e-03 * motor[motor_id].cal_current * motor[motor_id].cal_current;
    return motor_power;
}

//处理功率计反馈数据
void power_data_callback(CANRxData *Rx_data, CAN_PORT can_port)
{
    power_receive_data.pm_voltage = (float)((int32_t)(Rx_data->data[1]<<8) | (int32_t)(Rx_data->data[0]))/100.0;
    power_receive_data.pm_current = (float)((int32_t)(Rx_data->data[3]<<8) | (int32_t)(Rx_data->data[2]))/100.0;
    power_receive_data.pm_power = power_receive_data.pm_voltage * power_receive_data.pm_current;
}

// //计算单个电机功率
// float motor_power_clac(MotorPower *motor, uint8_t motor_id)
// {
//     float motor_power = 0;
//     motor_power = -2.0e-06 * motor[motor_id].current_current
//     -4.0e-6 *  motor[motor_id].current_speed + 0.7213;
//     return motor_power;
// }

//超级电容反馈
void super_cap_callback(CANRxData *Rx_data, CAN_PORT can_port)
{
    super_cap.capacitor_voltage = (Rx_data->data[0]<<8|Rx_data->data[1])/1000.0f;
    super_cap.capacitor_current = (int16_t)(Rx_data->data[2]<<8|Rx_data->data[3])/1000.0f;
    super_cap.battery_power = Rx_data->data[4];
    super_cap.limit_power = Rx_data->data[5];
    super_cap.capacitor_level = Rx_data->data[6];
}

//超级电容限制功率发送
void super_cap_limitpower_sent(FDCAN_HandleTypeDef *hfdcan, float power)
{
    uint8_t data[8] = {0};
    uint16_t can_id = 0x0FF;
    data[0] = (uint8_t)power;
    data[1] = (uint8_t)(power * 100.0f) % 100;
    fdcanx_send_data(hfdcan, can_id, data, 8);
}
