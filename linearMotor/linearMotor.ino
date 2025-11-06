#include <SPI.h>
#include "mcp_can.h"

const int SPI_CS_PIN = 53;
MCP_CAN CAN(SPI_CS_PIN);

void setup() {
  Serial.begin(115200);
  Serial.println("Init begin\n");
  if (CAN_OK == CAN.begin(MCP_STDEXT, CAN_1000KBPS, MCP_8MHZ)) {
    Serial.println("CAN init OK!\n");
  } else {
    Serial.println("CAN init FAILED!\n");
    while (1);
  }

  // delay(100);

  // // 发送 NMT 启动命令给 Node ID 1
  // unsigned char nmt[2] = {0x01, 0x01};
  // CAN.sendMsgBuf(0x000, 0, 2, nmt);
  // Serial.println("Sent NMT Start command");
}

void loop() {
  CAN.checkReceive()


}