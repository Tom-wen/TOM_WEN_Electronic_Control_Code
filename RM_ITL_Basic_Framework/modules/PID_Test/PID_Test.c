#include "PID_Test.h"
// #include "Init.h"
// #include <string.h>
// //Kp(1), Ki(2), Kd(3), Kf(4), IMax(5), OUT_Max(6), Dt(7), speedA(8), speedB(9), threshold(10)
// float PID_K[3][10] = {
//     {0.0, 0.0, 0.0, 0.0, 50.0, 2500.0f, 0.001f, 0.0, 0.0, 0.0},
//     {0.0, 0.0, 0.0, 0.0, 50.0, 2500.0f, 0.001f, 0.0, 0.0, 0.0},
//     {0.0, 0.0, 0.0, 0.0, 50.0, 2500.0f, 0.001f, 0.0, 0.0, 0.0}
// };

// uint8_t RxData;            //定义接收的数据
// void PID_Vofa_Init(PID *pid, float __K_P, float __K_I, float __K_D, float __K_F, float __I_Out_Max, float __Out_Max, float __D_T, float __Dead_Zone, float __I_Variable_Speed_A, float __I_Variable_Speed_B, float __I_Separate_Threshold, Enum_PID_D_First __D_First)
// {
//     pid->K_P = __K_P;
//     pid->K_I = __K_I;
//     pid->K_D = __K_D;
//     pid->K_F = __K_F;
//     pid->I_Out_Max = __I_Out_Max;
//     pid->Out_Max = __Out_Max;
//     pid->D_T = __D_T;
//     pid->Dead_Zone = __Dead_Zone;
//     pid->I_Variable_Speed_A = __I_Variable_Speed_A;
//     pid->I_Variable_Speed_B = __I_Variable_Speed_B;
//     pid->I_Separate_Threshold = __I_Separate_Threshold;
//     pid->D_First = __D_First;
    
// }
// void PID_Test_Init(float (*PID_K)[10], MotorInstance *motors)  // 修改为正确的二维数组指针类型
// {
//     // 使用第一组PID参数进行初始化
//     PID_Vofa_Init(&motors[0].motor_data->pid[single_loop], PID_K[0][KP], PID_K[0][KI], PID_K[0][KD], PID_K[0][KF], PID_K[0][IMax],
//     PID_K[0][OUT_Max], PID_K[0][Dt], 0.2f, PID_K[0][SPEED_A], PID_K[0][SPEED_B], PID_K[0][Threshold], PID_D_First_ENABLE);

//     PID_Vofa_Init(&motors[0].motor_data->pid[cascade_inner], PID_K[1][KP], PID_K[1][KI], PID_K[1][KD], PID_K[1][KF], PID_K[1][IMax],
//     PID_K[1][OUT_Max], PID_K[1][Dt], 0.2f, PID_K[1][SPEED_A], PID_K[1][SPEED_B], PID_K[1][Threshold], PID_D_First_ENABLE);

//     PID_Vofa_Init(&motors[0].motor_data->pid[cascade_outer], PID_K[2][KP], PID_K[2][KI], PID_K[2][KD], PID_K[2][KF], PID_K[2][IMax],
//     PID_K[2][OUT_Max], PID_K[2][Dt], 0.2f, PID_K[2][SPEED_A], PID_K[2][SPEED_B], PID_K[2][Threshold], PID_D_First_ENABLE);
// }

// void pid_test(uint8_t *buf, uint16_t len)
// {
//     // 检查数据长度是否足够
//     if (len < 4) return; // 至少需要 K + 编号 + 类型 + M
    
//     // 检查是否是有效的 K...M 命令
//     if (buf[data_start] == 'K' && buf[len-1] == 'M')
//     {
//         // 查找数值部分的开始位置
//         int value_start = 0;
//         for (int i = data_tpye + 1; i < len-1; i++) {
//             if ((buf[i] >= '0' && buf[i] <= '9') || buf[i] == '.') {
//                 value_start = i;
//                 break;
//             }
//         }
        
//         // 如果没有找到数值部分，直接返回
//         if (value_start == 0) return;
        
//         // 提取数值字符串
//         char value_str[16] = {0};
//         int value_len = (len-1) - value_start;
//         if (value_len >= sizeof(value_str)) value_len = sizeof(value_str) - 1;
//         memcpy(value_str, &buf[value_start], value_len);
        
//         // 转换为 float
//         float new_value = atof(value_str);
        
//         // 获取电机编号
//         uint8_t motor_num = buf[data_num] - '0';
        
//         // 确保电机编号在有效范围内
//         if(motor_num > 2) return;
        
//         // 根据类型设置 PID 参数
//         switch (buf[data_tpye])
//         {
//             case 'P':
//                 PID_K[motor_num][KP] = new_value;
//                 break;
//             case 'I':
//                 PID_K[motor_num][KI] = new_value;
//                 break;
//             case 'D':
//                 PID_K[motor_num][KD] = new_value;
//                 break;
//             case 'F':
//                 PID_K[motor_num][KF] = new_value;  
//                 break;
//             case 'X':
//                 PID_K[motor_num][IMax] = new_value;
//                 break;
//             case 'O':
//                 PID_K[motor_num][OUT_Max] = new_value;
//                 break;
//             case 'T':
//                 PID_K[motor_num][Dt] = new_value;
//                 break;
//             case 'A':
//                 PID_K[motor_num][SPEED_A] = new_value; 
//                 break;
//             case 'B':
//                 PID_K[motor_num][SPEED_B] = new_value;
//                 break;
//             case 't':
//                 PID_K[motor_num][Threshold] = new_value; 
//                 break;                             
//             default:
//                 break;
//         }
//     }
// }
