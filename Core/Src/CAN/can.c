#include "../../Inc/CAN/can.h"
#include "../../Inc/CAN/mcp2515.h"

char *bufferReceive[64];
char *bufferTransmit[64];

extern SPI_HandleTypeDef hspi1;

void MY_CAN_Init(void)
{
  // Initialize GPIO for CAN (SPI) communication

  // set NSS pin high, while transmitting it should be low
  HAL_GPIO_WritePin(CAN_NSS_GPIO_PORT, CAN_NSS_PIN, GPIO_PIN_SET);

  MCP2515_Reset();
  HAL_Delay(10); // wait for reset to complete
  if (!MCP2515_Initialize())
  {
    printf("Failed to initialize MCP2515\r\n");
    return;
  }

  HAL_GPIO_WritePin(CAN_NSS_GPIO_PORT, CAN_NSS_PIN, GPIO_PIN_RESET);
  uint8_t cmd = 0xC0; // RESET
  HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
  HAL_GPIO_WritePin(CAN_NSS_GPIO_PORT, CAN_NSS_PIN, GPIO_PIN_SET);

  HAL_Delay(10);

  uint8_t val = MCP2515_ReadByte(MCP2515_CANSTAT);
  printf("CANSTAT=0x%02X\r\n", val);

  uint8_t stat = MCP2515_ReadByte(MCP2515_CANSTAT);
  printf("After reset, CANSTAT=0x%02X\r\n", stat);

  if ((stat & 0xE0) != 0x80)
  {
    printf("Not in config mode after reset!\r\n");
    return false;
  }

  if (!MCP2515_SetConfigMode())
  {
    printf("Failed to set MCP2515 to configuration mode\r\n");
    return;
  }
  printf("CAN Initialized\r\n");
}

void MY_CAN_Transmit(char *data, uint8_t len)
{
  // Transmit data over CAN (SPI) bus
  uint8_t idReg[4];
  // 填充标准 ID 或扩展 ID 到 idReg[0..3]，格式见 MCP2515 datasheet

  uint8_t sdata[8] = {0x11, 0x22, 0x33, 0x44}; // 要发送的4字节数据
  uint8_t dlc = 4;                             // 数据长度

  // 1. 把数据写进某个发送缓冲区（比如 TXB0）
  MCP2515_LoadTxSequence(MCP2515_LOAD_TXB0D0, idReg, dlc, sdata);

  // 2. 请求发送
  MCP2515_RequestToSend(MCP2515_RTS_TX0);
}

// callback when the interrupt pin is triggered (should receive a message)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INTERRUPT_PIN)
  {
    printf("CAN Interrupt Triggered\r\n");
    // Handle CAN interrupt
    // Add your CAN interrupt handling code here
  }
}