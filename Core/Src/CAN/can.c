#include "CAN/can.h"


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