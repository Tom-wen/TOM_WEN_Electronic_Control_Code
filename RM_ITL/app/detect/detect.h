#ifndef __DETECT_H__
#define __DETECT_H__

#include "main.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "remote_control.h"

#define SBUS_TIMEOUT_MS  100               // 超时时间100ms（根据遥控器数据率调整）
extern uint16_t error_code;
//错误码
// 定义电机类型
typedef enum
{
    normal_runing = 0x140,
    HardFault = 0x141,
    remote_offline = 0x142,
    can_busy = 0x143
} ERROR_CODE;

void detect_task(void *argument);
void find_error(uint16_t error);

#endif
