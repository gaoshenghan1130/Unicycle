#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>

// MCP2515 SPI commands
#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_BITMOD      0x05
#define MCP_READ_STATUS 0xA0

// location of important registers
#define CANCTRL 0x0F
#define CNF1    0x2A
#define CNF2    0x29
#define CNF3    0x28
#define TXB0CTRL 0x30
#define TXB0SIDH 0x31
#define TXB0D0   0x36

void MCP2515_Init(void);
void MCP2515_SendFrame(uint32_t id, uint8_t* data, uint8_t len);



#endif /* MCP2515_H */