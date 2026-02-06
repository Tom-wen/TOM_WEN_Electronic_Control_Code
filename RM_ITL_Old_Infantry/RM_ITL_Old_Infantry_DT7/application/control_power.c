#include "control_power.h"

void motor_power_init(MotorPower* motor, const motor_power_init_t* init_data) 
{
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

MotorPower chassis_3508_motor[4];
MotorPower chassis_6020_motor[4];

// 创建初始化数据
motor_power_init_t motor3508_init_data[4] = {
    {//步兵左前
        .k0 = -0.011101188773801894,
        .k1 = 23.91595460113854,
        .k2 = 0.00012305478495736283,
        .k3 = 0.00027388660150835055,
        .k4 = -0.21141856679155993,
        .k5 = -0.0000003595116507414637,
        .real_current_conversion = 819.2
    },
    {//步兵右前
        .k0 = 0.0003642944469923325,
        .k1 = 23.888694916715693,
        .k2 = -0.000006572722340139778,
        .k3 = -0.000129434954098613,
        .k4 = -0.22401693249824417,
        .k5 = -0.0000000024887768911259146,
        .real_current_conversion = 819.2
    },
    {//步兵右后
        .k0 = -0.000750358679501406,
        .k1 = 23.54617039147782,
        .k2 = 0.0000008219056898861078,
        .k3 = 0.0000350360250162841,
        .k4 = -0.24857986831512072,
        .k5 = -0.000000001847383205095368,
        .real_current_conversion = 819.2
    },
    {//步兵左后
        .k0 = -0.0021005951735197374,
        .k1 = 23.849436844659657,
        .k2 = -0.00004011679562995295,
        .k3 = 0.00008793379002135882,
        .k4 = -0.04929416764816348,
        .k5 = 0.00000009553788848337535,
        .real_current_conversion = 819.2
    },
    // {
    //     .k0 = 1.0,
    //     .k1 = 1.0,
    //     .k2 = 1.0,
    //     .k3 = 1.0,
    //     .k4 = 1.0,
    //     .k5 = 1.0,
    //     .real_current_conversion = 1000.0
    // },
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

void Chassis_Power_Init(void)
{
    for (int i = 0; i < 4; i++){
        motor_power_init(&chassis_3508_motor[i], &motor3508_init_data[i]);
    }
}

//总目标实现功率控制
void Chassis_Power_Total_Control(float total_power_limit, uint8_t motor3508_number, uint8_t motor6020_number) 
{ 
    Total_3508motor_Power_Control(total_power_limit, motor3508_number, motor6020_number);
    //Total_6020motor_Power_Control(total_power_limit, motor6020_number, motor3508_number);
}

float Chassis_3508_Get_Limited_Number(uint8_t motor_id) 
{
    if (motor_id >= 4) return 0.0f;
    return chassis_3508_motor[motor_id].decay_number;
}

float Chassis_6020_Get_Limited_Number(uint8_t motor_id) 
{
    if (motor_id >= 4) return 0.0f;
    return chassis_6020_motor[motor_id].decay_number;
}


// 获取电机真实电流的辅助函数
double motor_power_get_real_current(const MotorPower *chassis_motor, double current)
{
    double real_current = current / chassis_motor->Current_Conversion;
    return real_current;
}

// 计算电机功率的主要函数
double motor_power_update(MotorPower* motor, E_Predict_Status_Type predict_status,
                        E_CalMotorPower_Negative_Status_Type negative_status) 
{
    const double product = motor->current_current * motor->current_speed;
    double power_sign = 1.0;
    // 处理负功率状态
    // if (negative_status == E_enable_negative) {
    //     if (product < 0) {
    //         power_sign = -1.0;
    //     }
    // }
    // 转换并取绝对值
    motor->cal_current = fabs(motor_power_get_real_current(motor, motor->current_current));
    motor->cal_speed = fabs(motor->current_speed);
    // 计算功率值
    float power = (motor->k0 + motor->k1*motor->cal_current + motor->k2*motor->cal_speed + 
        motor->k3 * motor->cal_current * motor->cal_speed + motor->k4 * motor->cal_current * motor->cal_current 
        + motor->k5 * motor->cal_speed * motor->cal_speed); //* power_sign;
    // 根据预测状态存储功率值到相应字段
    if (predict_status == E_enable_predict) {
        motor->predict_power = power;
    } else if (predict_status == E_disabled_predict) {
        motor->feedback_power = power;
    } else if (predict_status == E_enable_not_limit_predict) {
        motor->predict_not_limit_power = power;
    }
    return power;
}

//计算底盘3508实际总功率
float Chassis_3508_Actual_Power_Calculate(MotorPower *chassis_3508_motor, uint8_t motor3508_number)
{
    float total_3508_power = 0;
    for (int i = 0; i < motor3508_number; i++){
        total_3508_power += motor_power_update(&chassis_3508_motor[i], E_disabled_predict, E_enable_negative);
    }
    return total_3508_power;
}

//计算底盘6020实际总功率
float Chassis_6020_Actual_Power_Calculate(MotorPower *chassis_6020_motor, uint8_t motor6020_number){ 
    float total_6020_power = 0;
    for (int i = 0; i < motor6020_number; i++){
        total_6020_power += motor_power_update(&chassis_6020_motor[i], E_disabled_predict, E_enable_negative);
    }
    return total_6020_power;
}

//根据实际功率占比对限制功率进行分配
void Chassis_Power_Total_Allocate(float total_power_limit, float need_power_limit[2], uint8_t motor3508_number, uint8_t motor6020_number){
    float total_3508_power = Chassis_3508_Actual_Power_Calculate(chassis_3508_motor, motor3508_number);
    float total_6020_power = Chassis_6020_Actual_Power_Calculate(chassis_6020_motor, motor6020_number);
    float allocate_3508_rate = total_3508_power / (total_3508_power + total_6020_power);
    float allocate_6020_rate = total_6020_power / (total_3508_power + total_6020_power);
    need_power_limit[0] = total_power_limit * allocate_3508_rate;
    need_power_limit[1] = total_power_limit * allocate_6020_rate;
}


//按照分配完的功率进行功率控制
void Total_3508motor_Power_Control(float total_power_limit, uint8_t motor3508_number, uint8_t motor6020_number)
{
    float need_power_limit[2]; 
    Chassis_Power_Total_Allocate(total_power_limit, need_power_limit, motor3508_number, motor6020_number);
    float actual_total_3508_power = Chassis_3508_Actual_Power_Calculate(chassis_3508_motor, motor3508_number);
    float each_motor3508_allocated_rate[motor3508_number];
    for (int i = 0; i < motor3508_number; i++){
        each_motor3508_allocated_rate[i] = motor_power_update(&chassis_3508_motor[i], E_disabled_predict, E_enable_negative) / actual_total_3508_power;    
    }
    if (actual_total_3508_power > need_power_limit[0]){
        actual_total_3508_power = need_power_limit[0];//进行功率限制
    }
    for (int j = 0; j < motor3508_number; j++){
        chassis_3508_motor[j].power_limit = each_motor3508_allocated_rate[j] * actual_total_3508_power;
        chassis_3508_motor[j].decay_number =motor_power_limiter(&chassis_3508_motor[j], motor3508_number);
    }
}


void Total_6020motor_Power_Control(float total_power_limit, uint8_t motor6020_number, uint8_t motor3508_number){
    float need_power_limit[2];
    Chassis_Power_Total_Allocate(total_power_limit, need_power_limit, motor6020_number, motor3508_number);
    float actual_total_6020_power = Chassis_6020_Actual_Power_Calculate(chassis_6020_motor, motor6020_number);
    float each_motor6020_allocated_rate[motor6020_number];
    for (int i = 0; i < motor6020_number; i++){
        each_motor6020_allocated_rate[i] = motor_power_update(&chassis_6020_motor[i], E_disabled_predict, E_enable_negative) / actual_total_6020_power;    
    }
    if (actual_total_6020_power > need_power_limit[1]){
        actual_total_6020_power = need_power_limit[1];//进行功率限制
    }
    for (int j = 0; j < motor6020_number; j++){
        chassis_6020_motor[j].power_limit = each_motor6020_allocated_rate[j] * actual_total_6020_power;
        chassis_6020_motor[j].decay_number = motor_power_limiter(&chassis_6020_motor[j], motor6020_number);
    }
}


double motor_power_limiter(MotorPower *chassis_motor, uint8_t number) {
    
    float power_limit = chassis_motor->power_limit;
    chassis_motor->desired_current = chassis_motor->current_current;
    if(chassis_motor->power_limit < 0){
        chassis_motor->desired_current = 0;
        motor_power_update(chassis_motor, E_disabled_predict, E_enable_negative);
        return 0.0;
    }
//    const float desired_current1 = *desired_current;
    const float real_desired_current = fabs(motor_power_get_real_current(chassis_motor, chassis_motor->desired_current));
    const float real_current_speed = fabs(chassis_motor->current_speed);


    float predicted_power = motor_power_update(chassis_motor, E_enable_predict, E_enable_negative);
    if (predicted_power < power_limit){
        motor_power_update(chassis_motor, E_disabled_predict, E_enable_negative);
        return 1.0;
    }

    const double a = chassis_motor->k4 * real_desired_current * real_desired_current;
    const double b = (chassis_motor->k1 + chassis_motor->k3 * real_current_speed) * real_desired_current;
    const double c = chassis_motor->k0 + chassis_motor->k2 * real_current_speed + chassis_motor->k5 * real_current_speed * real_current_speed - chassis_motor->power_limit;
    const double discriminant = b * b - 4 * a * c;

    if (fabs(a) < 1e-9){
        if (fabs(b) < 1e-9){
            if (c <= 1e-9){
                motor_power_update(chassis_motor, E_enable_predict, E_disabled_negative);
                return 1.0;
            }else {
                chassis_motor->desired_current = 0;
                motor_power_update(chassis_motor, E_enable_predict, E_disabled_negative);
            }
        }else {
            double k = -c / b;
            if(k < 0){
                chassis_motor->desired_current = 0;
                motor_power_update(chassis_motor, E_enable_predict, E_enable_negative);
                return 0.0;
            }
            if (k > 1){
                motor_power_update(chassis_motor, E_enable_predict, E_enable_negative);
                return 1.0;
            }
            motor_power_update(chassis_motor, E_enable_predict, E_disabled_negative);
            return k;
        }    
    }

    if (discriminant < 0){ 
        chassis_motor->desired_current = 0;
        motor_power_update(chassis_motor, E_enable_predict, E_enable_negative);
    }

    if (fabs(discriminant) < 1e-9){
        double k = -1.0 * b / (2.0 * a);
        if (k < 0){
            chassis_motor->desired_current = 0;
            motor_power_update(chassis_motor, E_enable_predict, E_enable_negative);
        }else if (k > 1){
            motor_power_update(chassis_motor, E_enable_predict, E_enable_negative);
            return 1.0;
        }
        chassis_motor->desired_current *= k;
        motor_power_update(chassis_motor, E_enable_predict, E_enable_negative);
        return k;
    }

    const double k1 = (-b - sqrt(discriminant)) / (2 * a);
    const double k2 = (-b + sqrt(discriminant)) / (2 * a); 

    if( (k1 > 0.0 && k1 < 1.0 ) && (k2 > 0.0 && k2 < 1.0) ){
        chassis_motor->desired_current *= max(k1, k2);
        motor_power_update(chassis_motor, E_enable_predict, E_enable_negative);
        return max(k1, k2);
    }else if((k1 > 0.0 && k1 < 1.0) && (k2 > 1 || k2 < 0.0)){
        chassis_motor->desired_current *= k1;
        motor_power_update(chassis_motor, E_enable_predict, E_enable_negative);
        return k1;
    }else if((k1 < 0.0 || k1 > 1.0) && (k2 > 0.0 && k2 < 1.0)){
        chassis_motor->desired_current *= k2;
        motor_power_update(chassis_motor, E_enable_predict, E_enable_negative);
        return k2;
    }else{
        chassis_motor->desired_current = 0;
        motor_power_update(chassis_motor, E_enable_predict, E_enable_negative);
        return 0.0;
    }

}

void power_allocation_by_error(double* motor_errors_array, 
                              double* result_array,
                              int array_size,
                              double total_power_limit,
                              double buffer_power_attenuation) {
    
#ifdef M_Enable_PowerCompensation
    total_power_limit *= (1 - M_Power_Compensation_Alpha);
#endif

    total_power_limit *= buffer_power_attenuation;

    if (array_size != 4) {
        for (int i = 0; i < 4; i++) {
            result_array[i] = 0.0;
        }
        return;
    }

    // 对所有误差值取绝对值
    for (int i = 0; i < 4; i++) {
        motor_errors_array[i] = fabs(motor_errors_array[i]);
    }

    if (total_power_limit <= 1e-9) {
        for (int i = 0; i < 4; i++) {
            result_array[i] = 0.0;
        }
        return;
    }

    // 计算总误差
    double total_error = motor_errors_array[0] + motor_errors_array[1] + 
                        motor_errors_array[2] + motor_errors_array[3];

    if (total_error <= M_Too_Small_AllErrors) {
        double equal_share = total_power_limit / 4;
        for (int i = 0; i < 4; i++) {
            result_array[i] = equal_share;
        }
        return;
    }

    if (total_power_limit < M_Motor_ReservedPower_Border) {
        for (int i = 0; i < 4; i++) {
            double ratio = motor_errors_array[i] / total_error;
            result_array[i] = ratio * total_power_limit;
        }
    } else {
        for (int j = 0; j < 4; j++) {
            double ratio = motor_errors_array[j] / (total_error - 4 * M_PerMotor_ReservedPower);
            result_array[j] = ratio * (total_power_limit - 4 * M_PerMotor_ReservedPower) + M_PerMotor_ReservedPower;
        }
    }
}


void chassis_power_manager_init(ChassisPowerManager* chassis, 
                               MotorPower* motor1, MotorPower* motor2, 
                               MotorPower* motor3, MotorPower* motor4) {
    chassis->chassis_motors[0] = motor1;
    chassis->chassis_motors[1] = motor2;
    chassis->chassis_motors[2] = motor3;
    chassis->chassis_motors[3] = motor4;
    
    for (int i = 0; i < 4; i++) {
        chassis->motor_errors[i] = 0.0;
    }
}

// 更新电机误差
void chassis_power_manager_update_motor_error(ChassisPowerManager* chassis, 
                                            size_t index, double error) {
    if (index >= 4) return;
    chassis->motor_errors[index] = fabs(error);
}

// 分配功率
void chassis_power_manager_allocate_power(ChassisPowerManager* chassis,
                                        double total_power_limit,
                                        double buffer_power_attenuation) {
    // 注意：这里需要实现 power_allocation_by_error 的 C 版本
    double errors_array[4];
    double allocated_power[4];
    
    for (int i = 0; i < 4; i++) {
        errors_array[i] = chassis->motor_errors[i];
    }
    
    // 调用 C 版本的功率分配函数
    power_allocation_by_error(errors_array, allocated_power, 4, 
                           total_power_limit, buffer_power_attenuation);
    
    for (int i = 0; i < 4; i++) {
        chassis->chassis_motors[i]->power_limit = allocated_power[i];
    }
}

// 获取总预测未限制功率
double chassis_power_manager_get_total_predict_not_limit_power(const ChassisPowerManager* chassis) {
    double total = 0.0;
    for (int i = 0; i < 4; i++) {
        total += chassis->chassis_motors[i]->predict_not_limit_power;
    }
    return total;
}

// 获取总功率限制
double chassis_power_manager_get_total_power_limit(const ChassisPowerManager* chassis) {
    double total = 0.0;
    for (int i = 0; i < 4; i++) {
        total += chassis->chassis_motors[i]->power_limit;
    }
    return total;
}

// 获取总预测功率
double chassis_power_manager_get_total_predict_power(const ChassisPowerManager* chassis) {
    double total = 0.0;
    for (int i = 0; i < 4; i++) {
        total += chassis->chassis_motors[i]->predict_power;
    }
    return total;
}

double rotate_speed_allocation(int16_t vx, int16_t vy, int16_t rotate, double alpha) {
    // 计算平移速度
    const double translation = sqrt(vx*vx + vy*vy);
    double adjusted_rotate = abs(rotate) - alpha * translation;
    
    // 如果没有明显平移运动，则保持原旋转命令
    if (fabs(translation) <= 1e-9) {
        return rotate;
    }

    // 如果调整后旋转小于零，则设为0
    if (adjusted_rotate < 0) {
        adjusted_rotate = 0;
        return adjusted_rotate;
    }

    // 恢复符号
    if (rotate < 0) {
        return -adjusted_rotate;
    }
    return adjusted_rotate;
}

void rotate_theta_forwardfeed(double* theta, double rotate, double translation, double kp) {
    // 简易补偿
    if (translation != 0) {
        *theta = *theta - kp * rotate;
    }
}

// 移动平均滤波器结构体
typedef struct {
    double* buffer;
    size_t size;
    size_t index;
    size_t count;
    double sum;
} MovingAverageFilter;

int moving_average_filter_init(MovingAverageFilter* filter, size_t size) {
    filter->buffer = (double*)calloc(size, sizeof(double));
    if (filter->buffer == NULL) {
        return 0; // 初始化失败
    }
    
    filter->size = size;
    filter->index = 0;
    filter->count = 0;
    filter->sum = 0.0;
    
    return 1; // 初始化成功
}

double moving_average_filter_update(MovingAverageFilter* filter, double new_value) {
    filter->sum -= filter->buffer[filter->index];
    filter->buffer[filter->index] = new_value;
    filter->sum += new_value;
    filter->index = (filter->index + 1) % filter->size;
    
    if (filter->count < filter->size) {
        filter->count++;
    }
    
    return filter->sum / filter->count;
}
void moving_average_filter_destroy(MovingAverageFilter* filter) {
    if (filter->buffer != NULL) {
        free(filter->buffer);
        filter->buffer = NULL;
    }
}

double calculate_current_from_power(MotorPower* motor, double power, double speed){
    double a = motor->k4;
    double b = motor->k1 + motor->k3 * speed;
    double c = motor->k0 + motor->k2 * speed + motor->k5 * speed * speed - power;
    
    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
        return 0.0;
    }

    float current1 = (-b + sqrt(discriminant)) / (2 * a);
    float current2 = (-b - sqrt(discriminant)) / (2 * a);

    if(current1 >= 0 && current2 >= 0){
        return fmax(current1, current2);
    } else if (current1 >= 0){
        return current1;
    } else if (current2 >= 0){
        return current2;
    } else {
        return 0.0;
    }
}

double max(double a, double b)
{
    if(a >= b)
    {
        return a;
    }
    else
    {
        return b;
    }
}


// void Chassis_Power_Total_Allocate(MotorPower *chassis_motor, E_Control_Target_Type control_target){
//     float total_power_limit = 0;
//     uint8_t chassis_number = 4;
//     float buffer_power_attenuation = 1.0;
//     float allocated_power[chassis_number];
//     if (control_target == E_control_hero){
//         chassis_number = 4;
//         Chassis_Power_Servo_Allocate(allocated_power, total_power_limit, 0.3, chassis_power_manager_get_total_predict_not_limit_power(chassis_motor));
//         for(int i = 0; i < chassis_number; i++){
//             chassis_motor[i].power_limit = allocated_power[i];
//         }
//         Chassis_Power_Wheel_Allocate(chassis_motor, chassis_number);
//     } else if (control_target == E_control_infantry){
//         chassis_number = 4;
//         buffer_power_attenuation = 0.9;
//     } else if (control_target == E_control_engineer){
//         chassis_number = 4;
//         buffer_power_attenuation = 0.85;
//     } else if (control_target == E_control_sentry){
//         chassis_number = 4;
//         buffer_power_attenuation = 0.8;
//     }
//     //power_allocation_by_error(each_power_limitrt, allocated_power, 4, total_power_limit, buffer_power_attenuation);
// }

// void Chassis_Power_Wheel_Allocate(MotorPower *chassis_motor, uint8_t number){ 
//     float total_power_limit = 0;
//     float buffer_power_attenuation = 1.0;
//     float allocated_power[number];
//     float outputresult[number];    
//     for (int i = 0; i < number; i++){
//         outputresult[i] = motor_power_update(&chassis_motor[i], chassis_motor[i].current_current, chassis_motor[i].current_speed, E_enable_predict, E_enable_negative);
        
//         total_power_limit += outputresult[i];
//     }    
//     float desired_total_power_limit = chassis_power_manager_get_total_power_limit(chassis_motor);
//     float each_chassis_power_limitrt[number];
//     for (int i  = 0; i < number; i++){
//         each_chassis_power_limitrt[i] = desired_total_power_limit / number;
//         float current_need = calculate_current_from_power(&chassis_motor[i], each_chassis_power_limitrt[i], chassis_motor[i].current_speed);
//         chassis_motor[i].desired_current = current_need;
//         chassis_motor[i].decay_number = motor_power_limiter(&chassis_motor[i], &chassis_motor[i].desired_current, chassis_motor[i].current_speed, each_chassis_power_limitrt[i], number);
//     }
// }

// void Chassis_Power_Servo_Allocate(float total_power_limit, float servo_rate, float servo_predict_want_power, uint8_t number){ 
//     float total_servo_power_limit = total_power_limit * servo_rate;
//     float servo_motor_current[number];
//     MotorPower servo_motor[number];
//     float total_servo_power_limit;
//     for(int i = 0; i < number; i++){
//         servo_motor_current[i] = motor_power_get_real_current(&servo_motor[i], servo_motor[i].current_current);
//     }
//     for (int i = 0; i < number; i++){
//         total_servo_power_limit += motor_power_update(&servo_motor[i], servo_motor[i].current_current, servo_motor[i].current_speed, E_enable_predict, E_enable_negative);
//     }

// }

