#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(53);

struct can_frame frame;

void writeSDO(uint16_t index, uint8_t sub, uint32_t value) {
  struct can_frame f;
  f.can_id = 0x600 + 1;  // nodeID = 1
  f.can_dlc = 8;
  f.data[0] = 0x23;      // expedited download, 4 bytes
  f.data[1] = index & 0xFF;
  f.data[2] = index >> 8;
  f.data[3] = sub;
  f.data[4] = value & 0xFF;
  f.data[5] = (value >> 8) & 0xFF;
  f.data[6] = (value >> 16) & 0xFF;
  f.data[7] = (value >> 24) & 0xFF;

  mcp2515.sendMessage(&f);
}

void setup() {
  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);

  // receive ALL frames
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x000);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0x000);
  for (int i = 0; i < 6; i++) {
    mcp2515.setFilter((MCP2515::RXF)i, false, 0x000);
  }

  mcp2515.setNormalMode();

  Serial.println("Starting MC3001...");

  // (1) set operation mode = 1 (profile position mode)
  writeSDO(0x6060, 0x00, 1);
  delay(20);

  // (2) controlword sequence
  writeSDO(0x6040, 0x00, 0x0006);
  delay(20);
  writeSDO(0x6040, 0x00, 0x0007);
  delay(20);
  writeSDO(0x6040, 0x00, 0x000F);
  delay(20);
}

void loop() {
  struct can_frame rx;
  if (mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
    Serial.print("ID: ");
    Serial.print(rx.can_id, HEX);
    Serial.print(" Data: ");
    for (int i = 0; i < rx.can_dlc; i++) {
      Serial.print(rx.data[i], HEX); Serial.print(" ");
    }
    Serial.println();
  }
}