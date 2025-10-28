#include "../../Inc/CAN/can.h"
#include "../../Inc/CAN/mcp2515.h"
#include <stdint.h>
#include <stdio.h>

char *bufferReceive[64];
char *bufferTransmit[64];

volatile MotorFeedback motor_feedback; // only have one motor for now

extern SPI_HandleTypeDef hspi1;

void MY_CAN_Init(void);
void MY_CAN_Transmit(uint8_t *data, uint8_t len, uint8_t can_id);
void Motor_Init(uint8_t motor_id);
void Motor_SendTorque(uint16_t can_id, float torque, float torque_max);
MotorFeedback Motor_ParseFeedback(uint8_t *data, float Pmax, float Vmax, float Imax);

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
  // Bit timing (formula: Tq = 2*(BRP+1)/Fosc, BitRate = 1/(Tq*(1+PropSeg+PS1+PS2)))
  // Filters: Accept all messages (no filtering)
  // Masks: Not used in this configuration
  // Interrupts: Enable receive interrupts for both buffers
  //////////////////////////////////////////////////////////////////////////////////////////
  // Set bit timing registers
  MCP2515_WriteByte(0x2A, 0x00); // CNF1
  MCP2515_WriteByte(0x29, 0x80); // CNF2
  MCP2515_WriteByte(0x28, 0x80); // CNF3

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

void MY_CAN_Transmit(uint8_t *data, uint8_t len, uint8_t can_id)
{
  uint8_t idReg[4];

  // Standard ID 0x100
  idReg[0] = (uint8_t)(can_id >> 3);          // SIDH = ID[10:3]
  idReg[1] = (uint8_t)((can_id & 0x07) << 5); // SIDL = ID[2:0] << 5
  idReg[2] = 0x00;                            // EID8
  idReg[3] = 0x00;                            // EID0

  uint8_t dlc = len > 8 ? 8 : len; // DLC max 8 bytes

  // use library function to load the transmit buffer
  MCP2515_LoadTxSequence(MCP2515_LOAD_TXB0SIDH, idReg, dlc, data);

  // request to send the message
  MCP2515_RequestToSend(MCP2515_RTS_TX0);

  printf("CAN frame sent: ID=0x%03X, %d bytes\r\n", can_id, dlc);
}

// callback when the interrupt pin is triggered (should receive a message)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint8_t updatecount = 0;
  if (GPIO_Pin == INTERRUPT_PIN)
  {
    printf("CAN Interrupt Triggered\r\n");
    motor_feedback = Motor_ParseFeedback((uint8_t *)bufferReceive, 360.0f, 100.0f, 20.0f);
    updatecount++;
    motor_feedback.updated = updatecount > 1; // after the first update, it is considered updated
  }
}

///// Motor Message Senders //////////////////////////////////////////////////////////////////////////////////

void Motor_Init(uint8_t motor_id)
{
  printf("Initializing motor with CAN ID %d...\r\n", MOTOR_DEFAULT_ID);
  HAL_Delay(1000);

  MY_CAN_Init();
  uint8_t data[8];
  // first stop the motor with 0xFF..FD
  for (int i = 0; i < 8; i++)
    data[i] = 0xFF;
  data[7] = 0xFD;
  MY_CAN_Transmit(data, 8, MOTOR_DEFAULT_ID);
  printf("Motor[%d]: stopped.\r\n", MOTOR_DEFAULT_ID);

  HAL_Delay(100);

  // 0xFF..FE is used to set position to zero
  for (int i = 0; i < 8; i++)
    data[i] = 0xFF;
  data[7] = 0xFE;
  MY_CAN_Transmit(data, 8, MOTOR_DEFAULT_ID);
  printf("Motor[%d]: position set to zero.\r\n", MOTOR_DEFAULT_ID);

  HAL_Delay(100);

  // 0xFF..FA is used to set to torque mode
  for (int i = 0; i < 8; i++)
    data[i] = 0xFF;
  data[7] = 0xFA;
  MY_CAN_Transmit(data, 8, MOTOR_DEFAULT_ID);
  printf("Motor[%d]: set to torque mode.\r\n", MOTOR_DEFAULT_ID);

  HAL_Delay(100);

  // // 2. extended mode, usually not used
  // if (use_extended_mode)
  // {
  //     for (int i = 0; i < 8; i++) data[i] = 0x00;
  //     MY_CAN_Transmit(data, 8, motor_id);
  //     printf("Motor[%d]: switched to extended mode.\r\n", motor_id);
  // }

  Motor_SendTorque(MOTOR_DEFAULT_ID, 1.0f, 20.0f); // try 10A torque

  // 3. Start the motor using 0xFF..FC
  for (int i = 0; i < 8; i++)
    data[i] = 0xFF;
  data[7] = 0xFC;
  MY_CAN_Transmit(data, 8, MOTOR_DEFAULT_ID);
  printf("Motor[%d]: started.\r\n", MOTOR_DEFAULT_ID);
  HAL_Delay(100);
  Motor_SendTorque(MOTOR_DEFAULT_ID, 1.0f, 20.0f); // try 10A torque
}

// torque: expected torque value (A or Nm)
// torque_max: full scale torque value (A or Nm)
void Motor_SendTorque(uint16_t can_id, float torque, float torque_max)
{
  uint8_t data[8] = {0};

  // 1. 限幅
  if (torque > torque_max)
    torque = torque_max;
  if (torque < -torque_max)
    torque = -torque_max;

  // 2. transform to 12-bit code
  // [-torque_max, torque_max] → [0x000, 0xFFF] ，中点 0x800
  int16_t torque_code = (int16_t)((torque / torque_max) * 0x800 + 0x800);

  if (torque_code < 0)
    torque_code = 0;
  if (torque_code > 0xFFF)
    torque_code = 0xFFF;

  // 3. fill data array
  data[6] = (uint8_t)(torque_code >> 8) & 0x0F; // the high 4 bits of torque
  data[7] = (uint8_t)(torque_code & 0xFF);      // the low 8 bits of torque

  // 4. 发送
  MY_CAN_Transmit(data, 8, can_id);

  printf("Send torque=%.2f (code=0x%03X) to motor (CAN ID=0x%03X)\n",
         torque, torque_code, can_id);
}

MotorFeedback Motor_ParseFeedback(uint8_t *data, float Pmax, float Vmax, float Imax)
{
  MotorFeedback fb;
  uint16_t pos_code, vel_code, torque_code;

  // 位置：16bit
  pos_code = ((uint16_t)data[1] << 8) | data[2];

  // 速度：12bit
  vel_code = ((uint16_t)data[3] << 8) | (data[4] >> 4);

  // 力矩：12bit
  torque_code = (((uint16_t)(data[4] & 0x0F)) << 8) | data[5];

  // 解码
  fb.position_deg = ((int32_t)pos_code - 0x8000) / 32768.0f * 360.0f * Pmax;
  fb.velocity_rad = ((int32_t)vel_code - 0x800) / 2048.0f * Vmax;
  fb.torque_A = ((int32_t)torque_code - 0x800) / 2048.0f * Imax;

  return fb;
}