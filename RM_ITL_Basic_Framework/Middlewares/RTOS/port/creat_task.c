#include "Init.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* 任务函数外部声明 (弱定义在Init.c中，强定义在各模块文件中) */
extern void arm_task(void *argument);
extern void ins_task(void *argument);
extern void chassis_task(void *argument);
extern void detect_task(void *argument);
extern void gimbal_task(void *argument);
extern void data_task(void *argument);
extern void test_task(void *argument);
extern void shoot_task(void *argument);
extern void referee_task(void *argument);
extern void arm_lift_task(void *argument);
extern void hoisting_task(void *argument);
extern void lerobot_task(void *argument);
extern void ui_task(void *argument);

static void StartDefaultTask(void *argument);

/* ==================== 任务句柄 ==================== */
static TaskHandle_t defaultTaskHandle;
static TaskHandle_t armTaskHandle;
static TaskHandle_t insTaskHandle;
static TaskHandle_t chassisTaskHandle;
static TaskHandle_t detectTaskHandle;
static TaskHandle_t gimbalTaskHandle;
static TaskHandle_t dataTaskHandle;
static TaskHandle_t testTaskHandle;
static TaskHandle_t shootTaskHandle;
static TaskHandle_t refereeTaskHandle;
static TaskHandle_t armLiftTaskHandle;
static TaskHandle_t hoistingTaskHandle;
static TaskHandle_t lerobotTaskHandle;
static TaskHandle_t uiTaskHandle;

/* ==================== 静态任务控制块和栈缓冲区 ==================== */

/* app_arm: 512 words, osPriorityNormal → 优先级2 */
static StaticTask_t armTaskTCB;
static uint32_t armTaskBuffer[512];

/* app_ins: 1024 words, osPriorityHigh → 优先级4 */
static StaticTask_t insTaskTCB;
static uint32_t insTaskBuffer[1024];

/* app_chassis: 512 words, osPriorityNormal → 优先级2 */
static StaticTask_t chassisTaskTCB;
static uint32_t chassisTaskBuffer[512];

/* app_detect: 512 words, osPriorityHigh → 优先级4 */
static StaticTask_t detectTaskTCB;
static uint32_t detectTaskBuffer[512];

/* app_gimbal: 512 words, osPriorityHigh → 优先级4 */
static StaticTask_t gimbalTaskTCB;
static uint32_t gimbalTaskBuffer[512];

/* app_data: 512 words, osPriorityNormal → 优先级2 */
static StaticTask_t dataTaskTCB;
static uint32_t dataTaskBuffer[512];

/* app_test: 256 words, osPriorityNormal → 优先级2 */
static StaticTask_t testTaskTCB;
static uint32_t testTaskBuffer[256];

/* app_shoot: 512 words, osPriorityNormal → 优先级2 */
static StaticTask_t shootTaskTCB;
static uint32_t shootTaskBuffer[512];

/* app_referee: 512 words, osPriorityNormal → 优先级2 */
static StaticTask_t refereeTaskTCB;
static uint32_t refereeTaskBuffer[512];

/* app_arm_lift: 512 words, osPriorityNormal → 优先级2 */
static StaticTask_t armLiftTaskTCB;
static uint32_t armLiftTaskBuffer[512];

/* app_hoisting: 512 words, osPriorityNormal → 优先级2 */
static StaticTask_t hoistingTaskTCB;
static uint32_t hoistingTaskBuffer[512];

/* app_lerobot: 512 words, osPriorityNormal → 优先级2 */
static StaticTask_t lerobotTaskTCB;
static uint32_t lerobotTaskBuffer[512];

/* app_ui: 256 words, osPriorityNormal → 优先级2 */
static StaticTask_t uiTaskTCB;
static uint32_t uiTaskBuffer[256];

/* ==================== rs485 队列 (5项 x 4字节) ==================== */
QueueHandle_t rs485_queueHandle;
static StaticQueue_t rs485_queueTCB;
static uint8_t rs485_queueStorage[5 * 4];

/* ==================== Idle/Timer 任务静态内存 (静态分配必须提供) ==================== */
static StaticTask_t idleTaskTCB;
static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];
static StaticTask_t timerTaskTCB;
static StackType_t timerTaskStack[configTIMER_TASK_STACK_DEPTH];

/**
 * @brief FreeRTOS 任务和队列创建，启动调度器
 */
void RTOS_Init(void)
{
    /* 创建 rs485 消息队列 */
    rs485_queueHandle = xQueueCreateStatic(5, 4, rs485_queueStorage, &rs485_queueTCB);

    /* defaultTask: 动态分配 (原始代码无静态缓冲区), MCP2515轮询 */
    xTaskCreate(StartDefaultTask, "defaultTask", 256, NULL, 2, &defaultTaskHandle);

    /* 静态任务创建 */
    armTaskHandle      = xTaskCreateStatic(arm_task,      "app_arm",      512,  NULL, 2, armTaskBuffer,      &armTaskTCB);
    insTaskHandle      = xTaskCreateStatic(ins_task,      "app_ins",     1024,  NULL, 4, insTaskBuffer,      &insTaskTCB);
    chassisTaskHandle  = xTaskCreateStatic(chassis_task,  "app_chassis",  512,  NULL, 2, chassisTaskBuffer,  &chassisTaskTCB);
    detectTaskHandle   = xTaskCreateStatic(detect_task,   "app_detect",   512,  NULL, 4, detectTaskBuffer,   &detectTaskTCB);
    gimbalTaskHandle   = xTaskCreateStatic(gimbal_task,   "app_gimbal",   512,  NULL, 4, gimbalTaskBuffer,   &gimbalTaskTCB);
    dataTaskHandle     = xTaskCreateStatic(data_task,     "app_data",     512,  NULL, 2, dataTaskBuffer,     &dataTaskTCB);
    testTaskHandle     = xTaskCreateStatic(test_task,     "app_test",     256,  NULL, 2, testTaskBuffer,     &testTaskTCB);
    shootTaskHandle    = xTaskCreateStatic(shoot_task,    "app_shoot",    512,  NULL, 2, shootTaskBuffer,    &shootTaskTCB);
    refereeTaskHandle  = xTaskCreateStatic(referee_task,  "app_referee", 512,  NULL, 2, refereeTaskBuffer,  &refereeTaskTCB);
    armLiftTaskHandle  = xTaskCreateStatic(arm_lift_task, "app_arm_lift", 512,  NULL, 2, armLiftTaskBuffer,  &armLiftTaskTCB);
    hoistingTaskHandle = xTaskCreateStatic(hoisting_task, "app_hoisting", 512,  NULL, 2, hoistingTaskBuffer, &hoistingTaskTCB);
    lerobotTaskHandle  = xTaskCreateStatic(lerobot_task,  "app_lerobot",  512,  NULL, 2, lerobotTaskBuffer,  &lerobotTaskTCB);
    uiTaskHandle       = xTaskCreateStatic(ui_task,       "app_ui",      256,  NULL, 2, uiTaskBuffer,       &uiTaskTCB);

    /* 启动调度器 */
    vTaskStartScheduler();
}

/**
 * @brief 默认任务: MCP2515 SPI转CAN轮询
 */
static void StartDefaultTask(void *argument)
{
    (void)argument;
    for (;;)
    {
        MCP2515_Rx_Poll();
        MCP2515_2_Rx_Poll();
        MCP2515_3_Rx_Poll();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/**
 * @brief 静态分配必须实现: 提供Idle任务内存
 */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &idleTaskTCB;
    *ppxIdleTaskStackBuffer = idleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/**
 * @brief 静态分配必须实现: 提供Timer任务内存
 */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &timerTaskTCB;
    *ppxTimerTaskStackBuffer = timerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
