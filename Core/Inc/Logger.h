#include <string.h>

#define UART_LOG(msg)  do { \
    const char *_msg = (msg); \
    HAL_UART_Transmit(&huart1, (uint8_t*)_msg, strlen(_msg), HAL_MAX_DELAY); \
} while(0)