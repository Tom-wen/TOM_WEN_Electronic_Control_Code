#include "bsp_spi.h"
#include "bsp_can.h"
#include "FreeRTOS.h"
#include "task.h"

volatile uint8_t mcp2515_1_rx_flag = 0;
volatile uint8_t mcp2515_2_rx_flag = 0;
volatile uint8_t mcp2515_3_rx_flag = 0;

// SPI 读写辅助函数
static void SPI_Write(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t *txData, uint8_t size)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, txData, size, 100);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

static void SPI_Read(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint8_t *txData, uint8_t *rxData, uint8_t size)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, txData, rxData, size, 100);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

// 初始化 MCP2515 (f_osc=8MHz, 1Mbps)
void MCP2515_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint8_t txData[5];

    // 复位 MCP2515
    txData[0] = 0xC0;
    SPI_Write(hspi, GPIOx, GPIO_Pin, txData, 1);
    HAL_Delay(10);

    // 进入配置模式
    txData[0] = MCP2515_WRITE_CMD;
    txData[1] = 0x0F; // CANCTRL
    txData[2] = 0x80; // REQOP=100 配置模式
    SPI_Write(hspi, GPIOx, GPIO_Pin, txData, 3);
    HAL_Delay(10);

    // 从 0x28 开始连续写 CNF3, CNF2, CNF1（地址自增）
    txData[0] = MCP2515_WRITE_CMD;
    txData[1] = 0x28; // 起始地址
    txData[2] = 0x00; // CNF3 (0x28): PHSEG2=1
    txData[3] = 0x80; // CNF2 (0x29): BTLMODE=1, PHSEG1=1, PRSEG=1
    txData[4] = 0x00; // CNF1 (0x2A): BRP=0, SJW=1
    SPI_Write(hspi, GPIOx, GPIO_Pin, txData, 5);

    // 设置 RXB0 接收所有消息（关闭过滤器）
    txData[0] = MCP2515_WRITE_CMD;
    txData[1] = 0x60; // RXB0CTRL
    txData[2] = 0x60; // RXM=11, 接收所有消息
    SPI_Write(hspi, GPIOx, GPIO_Pin, txData, 3);

    // 使能 MCP2515 RX0 中断，收到消息时 INT 引脚拉低
    txData[0] = MCP2515_WRITE_CMD;
    txData[1] = 0x2B; // CANINTE
    txData[2] = 0x01; // RX0IE=1
    SPI_Write(hspi, GPIOx, GPIO_Pin, txData, 3);

    // 切换到正常模式
    txData[0] = MCP2515_WRITE_CMD;
    txData[1] = 0x0F; // CANCTRL
    txData[2] = 0x00; // 正常模式
    SPI_Write(hspi, GPIOx, GPIO_Pin, txData, 3);

    // 清除可能已置位的 RX0IF，确保 INT 引脚释放为高电平
    txData[0] = MCP2515_BITMOD_CMD;
    txData[1] = 0x2C; // CANINTF
    txData[2] = 0x01; // mask: RX0IF
    txData[3] = 0x00; // 清零
    SPI_Write(hspi, GPIOx, GPIO_Pin, txData, 4);
}

// 发送 CAN 帧
void MCP2515_Send(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t id, uint8_t *data, uint8_t len)
{
    uint8_t txData[14];
    uint8_t i;
    // 加载 TX 缓冲区 (使用 TXB0)
    txData[0] = MCP2515_LOAD_TX0_CMD;
    txData[1] = (id >> 3) & 0xFF; // SIDH
    txData[2] = (id << 5) & 0xE0; // SIDL
    txData[3] = 0x00;              // EID8 (标准帧填0)
    txData[4] = 0x00;              // EID0 (标准帧填0)
    txData[5] = len;               // DLC
    for (i = 0; i < len; i++) {
      txData[6 + i] = data[i];
    }
    SPI_Write(hspi, GPIOx, GPIO_Pin, txData, 6 + len);
    // 发送 RTS 命令
    txData[0] = MCP2515_RTS_CMD | 0x01; // RTS for TXB0
    SPI_Write(hspi, GPIOx, GPIO_Pin, txData, 1);
}

// 接收 CAN 帧
void MCP2515_Receive(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t *id, uint8_t *data, uint8_t *len) {
  uint8_t txBuf[15] = {0}, rxBuf[15];

  // 读 CANINTF (0x2C) 检查 RX0IF
  txBuf[0] = MCP2515_READ_CMD;
  txBuf[1] = 0x2C;
  SPI_Read(hspi, GPIOx, GPIO_Pin, txBuf, rxBuf, 3);

  if (!(rxBuf[2] & 0x01)) { *len = 0; return; }

  // 读 RXB0: 从 SIDH(0x61) 开始，共 13 字节寄存器 = SPI 15 字节
  txBuf[0] = MCP2515_READ_CMD;
  txBuf[1] = 0x61;
  SPI_Read(hspi, GPIOx, GPIO_Pin, txBuf, rxBuf, 15);

  *id = ((uint32_t)rxBuf[2] << 3) | (rxBuf[3] >> 5);
  *len = rxBuf[6] & 0x0F;
  for (uint8_t i = 0; i < *len; i++) {
    data[i] = rxBuf[7 + i];
  }

}

//清除RX0IF
void MCP2515_RX0IF_clear(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint8_t txBuf[4] = {0};
    txBuf[0] = MCP2515_BITMOD_CMD;
    txBuf[1] = 0x2C;
    txBuf[2] = 0x01;
    txBuf[3] = 0x00;
    SPI_Write(hspi, GPIOx, GPIO_Pin, txBuf, 4);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SPI1_INT1_Pin){
      mcp2515_1_rx_flag = 1;
    }
    else if (GPIO_Pin == SPI1_INT2_Pin){
      mcp2515_2_rx_flag = 1;
    }
    else if (GPIO_Pin == SPI1_INT3_Pin){
      mcp2515_3_rx_flag = 1;
    }
}

// CAN4 (MCP2515 #1) 接收轮询
void MCP2515_Rx_Poll(void)
{
    if (!mcp2515_1_rx_flag) return;
    mcp2515_1_rx_flag = 0;

    uint32_t id = 0;
    uint8_t data[8] = {0};
    uint8_t len = 0;

    taskENTER_CRITICAL();
    MCP2515_Receive(&hspi1, SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, &id, data, &len);
    MCP2515_RX0IF_clear(&hspi1, SPI1_CS1_GPIO_Port, SPI1_CS1_Pin);
    taskEXIT_CRITICAL();

    fdcan4_mcp2515_rx_dispatch(id, data, len);
}

// CAN5 (MCP2515 #2) 接收轮询
void MCP2515_2_Rx_Poll(void)
{
    if (!mcp2515_2_rx_flag) return;
    mcp2515_2_rx_flag = 0;

    uint32_t id = 0;
    uint8_t data[8] = {0};
    uint8_t len = 0;

    taskENTER_CRITICAL();
    MCP2515_Receive(&hspi1, SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, &id, data, &len);
    MCP2515_RX0IF_clear(&hspi1, SPI1_CS2_GPIO_Port, SPI1_CS2_Pin);
    taskEXIT_CRITICAL();

    fdcan5_mcp2515_rx_dispatch(id, data, len);
}

// CAN6 (MCP2515 #3) 接收轮询
void MCP2515_3_Rx_Poll(void)
{
    if (!mcp2515_3_rx_flag) return;
    mcp2515_3_rx_flag = 0;

    uint32_t id = 0;
    uint8_t data[8] = {0};
    uint8_t len = 0;

    taskENTER_CRITICAL();
    MCP2515_Receive(&hspi1, SPI1_CS3_GPIO_Port, SPI1_CS3_Pin, &id, data, &len);
    MCP2515_RX0IF_clear(&hspi1, SPI1_CS3_GPIO_Port, SPI1_CS3_Pin);
    taskEXIT_CRITICAL();

    fdcan6_mcp2515_rx_dispatch(id, data, len);
}
