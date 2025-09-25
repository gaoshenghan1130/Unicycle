#ifndef CAN_H
#define CAN_H

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32wbxx_hal.h"


#define INTERRUPT_PIN        GPIO_PIN_0
#define INTERRUPT_GPIO_PORT  GPIOA
#define CAN_NSS_PIN        GPIO_PIN_5
#define CAN_NSS_GPIO_PORT  GPIOA


extern char* bufferReceive[64];
extern char* bufferTransmit[64];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



#endif /* CAN_H */