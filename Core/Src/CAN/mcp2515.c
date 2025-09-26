#include "../../Inc/CAN/can.h"
#include "../../Inc/CAN/mcp2515.h"
#include "../../Inc/CAN/can.h"

extern SPI_HandleTypeDef hspi1;

static void MCP2515_Select(void) {
    HAL_GPIO_WritePin(CAN_NSS_GPIO_PORT, CAN_NSS_PIN, GPIO_PIN_RESET);
}

static void MCP2515_Unselect(void) {
    HAL_GPIO_WritePin(CAN_NSS_GPIO_PORT, CAN_NSS_PIN, GPIO_PIN_SET);
}

static void MCP2515_WriteRegister(uint8_t addr, uint8_t val) {
    uint8_t buf[2] = {MCP_WRITE, addr};
    MCP2515_Select();
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, &val, 1, HAL_MAX_DELAY);
    MCP2515_Unselect();
}

static void MCP2515_Reset(void) {
    uint8_t cmd = MCP_RESET;
    MCP2515_Select();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    MCP2515_Unselect();
    HAL_Delay(10);
}

void MCP2515_Init(void) {
    MCP2515_Reset();

    // port rate 500kbps, assuming 16MHz clock
    MCP2515_WriteRegister(CNF1, 0x00);
    MCP2515_WriteRegister(CNF2, 0x90);
    MCP2515_WriteRegister(CNF3, 0x02);

    MCP2515_WriteRegister(CANCTRL, 0x00);
}

void MCP2515_SendFrame(uint32_t id, uint8_t* data, uint8_t len) {
    uint8_t sid_high = (id >> 3) & 0xFF;
    uint8_t sid_low  = (id << 5) & 0xE0;

    MCP2515_WriteRegister(TXB0SIDH, sid_high);
    MCP2515_WriteRegister(TXB0SIDH+1, sid_low);
    MCP2515_WriteRegister(TXB0SIDH+2, len & 0x0F); // DLC
    for (uint8_t i=0; i<len; i++) {
        MCP2515_WriteRegister(TXB0D0 + i, data[i]);
    }

    uint8_t rts = 0x81; // RTS TXB0
    MCP2515_Select();
    HAL_SPI_Transmit(&hspi1, &rts, 1, HAL_MAX_DELAY);
    MCP2515_Unselect();
}