#include "detect.h"
#include "bsp_led.h"
uint16_t error_code = 0x140;
void detect_task(void *argument)
{
    uint32_t current_time;
    for(;;)
    {
        #ifdef DJI_REMOTE
            current_time = xTaskGetTickCount();
            // 检查是否超时（遥控器离线检测）
            if(sbus_online && (current_time - last_sbus_recv_time > pdMS_TO_TICKS(SBUS_TIMEOUT_MS)))
            {
                sbus_online = 0;  // 标记离线
            }
        #endif
        #ifdef FS_REMOTE
            if(rc_ctrl.rc.frame_lost == 1)
            {
                sbus_online = 0;  // 标记离线
            }
        #endif
        #ifdef VT_03_REMOTE
        current_time = xTaskGetTickCount();
        // 检查是否超时（遥控器离线检测）
        if(sbus_online && (current_time - last_sbus_recv_time > pdMS_TO_TICKS(SBUS_TIMEOUT_MS)))
        {
            sbus_online = 0;  // 标记离线
        }
        #endif
        if(sbus_online == 0)
        {
            error_code = remote_offline;
        }
        else 
        {
            error_code = normal_runing;
        }
        //错误码更新
        find_error(error_code);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void find_error(uint16_t error)
{
    switch (error)
    {
    case normal_runing:
        WS2812_Ctrl(0,20,0);//绿色
        break;
    case HardFault:
        WS2812_Ctrl(20,0,0);//红色
        break;   
    case remote_offline:
        WS2812_Ctrl(0,0,20);//蓝色
    break;  
    case can_busy:
        WS2812_Ctrl(20,20,0);//黄色
    break;         
    default:
        break;
    }
}
