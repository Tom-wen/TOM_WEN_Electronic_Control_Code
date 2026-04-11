bsp_can

如果要添加其他CAN口直接在这里添加就行
## 给CAN口定义接收数组
CANRxData Rx_data1[CAN_MAX_SUPPORTED_ID];//CAN1接收数据
CANRxData Rx_data2[CAN_MAX_SUPPORTED_ID];//CAN2接收数据，支持多个ID

## 为每个CAN口声明独立的回调函数指针
static can_rx_callback_t can1_user_callback = NULL;//CAN1
static can_rx_callback_t can2_user_callback = NULL;//CAN2

## 为每个can口注册回调函数
void bsp_can1_set_callback(can_rx_callback_t callback)
void bsp_can2_set_callback(can_rx_callback_t callback)

## 使用方法
void bsp_can_init(void) 这是初始化函数，一般只要按相同格式就可以添加其他的CAN口
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len) 发送函数，通过这个函数来发送报文
void fdcan1_rx_callback(void)
void fdcan2_rx_callback(void) CAN回调函数，如果要增加CAN口需要加一个CAN回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) CAN中断回调函数，如果添加里CAN回调函数后需要再这里使用它
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) 错误处理，使用hal库自带的函数，原来是弱函数，所以不需要去调用它，这个函数会在CAN通信的FIFO满时会自动使用
来重新启动CAN
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) CAN BUS OFF后的的处理函数，会自动重启，去取消BUS OFF状态

