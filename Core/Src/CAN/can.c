#include "../../Inc/CAN/can.h"
#include "../../Inc/CAN/mcp2515.h"
#include "../../Inc/CAN/CANSPI.h"
#include "stm32wbxx_hal_gpio.h"
#include <stdint.h>
#include <stdio.h>

char *bufferReceive[64];
char *bufferTransmit[64];

volatile MotorFeedback motor_feedback; // only have one motor for now

extern SPI_HandleTypeDef hspi1;
uCAN_MSG* dataToCANSPIdata(uint8_t *data, uCAN_MSG *can_msg) {
  can_msg->frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  can_msg->frame.id = MOTOR_DEFAULT_ID;
  can_msg->frame.dlc = 8;
  can_msg->frame.data0 = data[0];
  can_msg->frame.data1 = data[1];
  can_msg->frame.data2 = data[2];
  can_msg->frame.data3 = data[3];
  can_msg->frame.data4 = data[4];
  can_msg->frame.data5 = data[5];
  can_msg->frame.data6 = data[6];
  can_msg->frame.data7 = data[7];
  return can_msg;
}
void MY_CAN_Init(void);
void Motor_Init(uint8_t motor_id);
void Motor_SendTorque(uint16_t can_id, float torque, float torque_max);
MotorFeedback Motor_ParseFeedback(uint8_t *data, float Pmax, float Vmax,
                                  float Imax);

void MY_CAN_Init(void) {
  HAL_GPIO_WritePin(CAN_NSS_GPIO_PORT, CAN_NSS_PIN, GPIO_PIN_RESET); // bring CS low forever
  CANSPI_Initialize();
}

void MY_CAN_Transmit(uint8_t *data, uint8_t len, int can_id) {
  uCAN_MSG can_msg[14];
  HAL_GPIO_WritePin(CAN_NSS_GPIO_PORT, CAN_NSS_PIN, GPIO_PIN_RESET); // bring CS low
  CANSPI_Transmit(dataToCANSPIdata(data, can_msg));
  HAL_GPIO_WritePin(CAN_NSS_GPIO_PORT, CAN_NSS_PIN, GPIO_PIN_SET); // bring CS high
}

void MY_CAN_Receive(uint8_t *data, uint8_t len, int can_id) {
  uCAN_MSG can_msg[14];
  HAL_GPIO_WritePin(CAN_NSS_GPIO_PORT, CAN_NSS_PIN, GPIO_PIN_RESET); // bring CS low
  CANSPI_Receive(dataToCANSPIdata(data, can_msg));
  HAL_GPIO_WritePin(CAN_NSS_GPIO_PORT, CAN_NSS_PIN, GPIO_PIN_SET); // bring CS high
}


// callback when the interrupt pin is triggered (should receive a message)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  static uint8_t updatecount = 0;
  if (GPIO_Pin == INTERRUPT_PIN) {
    //printf("CAN Interrupt Triggered\r\n");
    motor_feedback =
        Motor_ParseFeedback((uint8_t *)bufferReceive, 360.0f, 100.0f, 20.0f);
    updatecount++;
    motor_feedback.updated =
        updatecount > 1; // after the first update, it is considered updated
  }
}

///// Motor Message Senders
/////////////////////////////////////////////////////////////////////////////////////

void Motor_Init(uint8_t motor_id) {
  printf("Initializing motor with CAN ID %d...\r\n", MOTOR_DEFAULT_ID);

  MY_CAN_Init();
  uint8_t data[8];
  // first stop the motor with 0xFF..FD
  for (int i = 0; i < 8; i++)
    data[i] = 0xFF;
  data[7] = 0xFD;
  uCAN_MSG can_msg[14];
  MY_CAN_Transmit(data, 8, MOTOR_DEFAULT_ID);

  HAL_Delay(100);

  // 0xFF..FE is used to set position to zero
  for (int i = 0; i < 8; i++)
    data[i] = 0xFF;
  data[7] = 0xFE;
  MY_CAN_Transmit(data, 8, MOTOR_DEFAULT_ID);

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
void Motor_SendTorque(uint16_t can_id, float torque, float torque_max) {
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

  // 4. transmit via CAN
  uCAN_MSG can_msg[14];
  CANSPI_Transmit(dataToCANSPIdata(data, can_msg));

  printf("Send torque=%.2f (code=0x%03X) to motor (CAN ID=0x%03X)\n", torque,
         torque_code, can_id);
}

MotorFeedback Motor_ParseFeedback(uint8_t *data, float Pmax, float Vmax,
                                  float Imax) {
  MotorFeedback fb;
  uint16_t pos_code, vel_code, torque_code;

  // 位置：16bit
  pos_code = ((uint16_t)data[1] << 8) | data[2];

  // 速度：12bit
  vel_code = ((uint16_t)data[3] << 4) | (data[4] >> 4);

  // 力矩：12bit
  torque_code = (((uint16_t)(data[4] & 0x0F)) << 8) | data[5];

  // 解码
  fb.position_deg = ((int32_t)pos_code - 0x8000) / 32768.0f * 360.0f * Pmax;
  fb.velocity_rad = ((int32_t)vel_code - 0x800) / 2048.0f * Vmax;
  fb.torque_A = ((int32_t)torque_code - 0x800) / 2048.0f * Imax;

  //printf("Motor Feedback: Pos=%.2f deg, Vel=%.2f rad/s, Torque=%.2f A\r\n", fb.position_deg, fb.velocity_rad, fb.torque_A);

  return fb;
}