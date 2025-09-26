#include "../../Inc/CAN/can.h"
#include "../../Inc/CAN/mcp2515.h"

char* bufferReceive[64];
char* bufferTransmit[64];

extern SPI_HandleTypeDef hspi1;


void MY_CAN_Init(void)
{
    // Initialize GPIO for CAN (SPI) communication

    // set NSS pin high, while transmitting it should be low
    HAL_GPIO_WritePin(CAN_NSS_GPIO_PORT, CAN_NSS_PIN, GPIO_PIN_SET);
    MCP2515_Init();
  printf("CAN Initialized\r\n");
}

void MY_CAN_Transmit(char* data, uint8_t len)
{
    // Transmit data over CAN (SPI) bus
    // Pull NSS low to select the CAN transceiver
    HAL_GPIO_WritePin(CAN_NSS_GPIO_PORT, CAN_NSS_PIN, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, (uint8_t*)data, len, HAL_MAX_DELAY);

    // Pull NSS high to deselect the CAN transceiver
    HAL_GPIO_WritePin(CAN_NSS_GPIO_PORT, CAN_NSS_PIN, GPIO_PIN_SET);
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