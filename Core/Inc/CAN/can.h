#ifndef CAN_H
#define CAN_H

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32wbxx_hal.h"
#include "mcp2515.h"

// check document in ./Doc/Com/CAN_Bus.md for more details about pinout


#define INTERRUPT_PIN        GPIO_PIN_0
#define INTERRUPT_GPIO_PORT  GPIOA



extern char* bufferReceive[64];
extern char* bufferTransmit[64];

typedef struct {
    float position_deg;   // position, degrees
    float velocity_rad;   // velocity, rad/s
    float torque_A;       // torque, A(current)
} MotorFeedback;

void Motor_Init(uint8_t motor_id);
void Motor_SendTorque(uint16_t can_id, float torque, float torque_max);

#endif /* CAN_H */