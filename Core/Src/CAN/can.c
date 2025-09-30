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

  // Examine SPI connection
  if (!MCP2515_Initialize())
  {
    printf("Failed to initialize MCP2515\r\n");
    return;
  }

  // Enter configuration mode
  if (!MCP2515_SetConfigMode())
  {
    printf("Failed to set MCP2515 to configuration mode\r\n");
    return;
  }

  // Configurations ////////////////////////////////////////////////////////////////////////
  // Port rate: 1Mbps
  // Bit timing: SJW=1, BRP=0, PropSeg=1, PS1=3, PS2=2  (formula: Tq = 2*(BRP+1)/Fosc, BitRate = 1/(Tq*(1+PropSeg+PS1+PS2)))
  // Filters: Accept all messages (no filtering)
  // Masks: Not used in this configuration
  // Interrupts: Enable receive interrupts for both buffers
  //////////////////////////////////////////////////////////////////////////////////////////
  // Set bit timing registers
  MCP2515_WriteByte(0x2A, 0x00); // CNF1
  MCP2515_WriteByte(0x29, 0x90); // CNF2
  MCP2515_WriteByte(0x28, 0x02); // CNF3
  // 0.875e-6s per bit, 1.1Mbps, close enough to 1Mbps

  // Set RX filters to accept all messages
  MCP2515_WriteByte(0x00, 0x00); // RXF0SIDH
  MCP2515_WriteByte(0x01, 0x00); // RXF0SIDL
  MCP2515_WriteByte(0x04, 0x00); // RXF1SIDH
  MCP2515_WriteByte(0x05, 0x00); // RXF1SIDL
  MCP2515_WriteByte(0x08, 0x00); // RXF2SIDH
  MCP2515_WriteByte(0x09, 0x00); // RXF2SIDL
  MCP2515_WriteByte(0x10, 0x00); // RXF3SIDH
  MCP2515_WriteByte(0x11, 0x00); // RXF3SIDL
  MCP2515_WriteByte(0x14, 0x00); // RXF4SIDH
  MCP2515_WriteByte(0x15, 0x00); // RXF4SIDL
  MCP2515_WriteByte(0x18, 0x00); // RXF5SIDH
  MCP2515_WriteByte(0x19, 0x00); // RXF5SIDL
  // Enable receive interrupts
  MCP2515_WriteByte(0x2B, 0x03); // CANINTE
  // Set RX buffer control registers to receive all messages
  MCP2515_WriteByte(0x60, 0x60); // RXB0CTRL
  MCP2515_WriteByte(0x70, 0x60); // RXB1CTRL
  //////////////////////////////////////////////////////////////////////////////////////////
  // Enter normal mode
  if (!MCP2515_SetNormalMode())
  {
    printf("Failed to set MCP2515 to normal mode\r\n");
    return;
  }
  printf("CAN Initialized\r\n");
}

void MY_CAN_Transmit(char *data, uint8_t len)
{
  uint8_t idReg[4];

  // 标准帧 ID = 0x100
  idReg[0] = 0x20; // SIDH = ID[10:3]
  idReg[1] = 0x00; // SIDL = ID[2:0] << 5
  idReg[2] = 0x00; // EID8
  idReg[3] = 0x00; // EID0

  uint8_t dlc = len > 8 ? 8 : len; // DLC 最大 8 字节

  // 使用库函数加载 TXB0
  MCP2515_LoadTxSequence(MCP2515_LOAD_TXB0SIDH, idReg, dlc, data);

  // 请求发送 TXB0
  MCP2515_RequestToSend(MCP2515_RTS_TX0);

  printf("CAN frame sent: ID=0x100, %d bytes\r\n", dlc);
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