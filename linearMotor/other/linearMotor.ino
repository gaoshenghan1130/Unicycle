#include <SPI.h>
#include "mcp_can.h"

#define EXPENDED_CAN 0  // not using expended can

const int SPI_CS_PIN = 53;
MCP_CAN CAN(SPI_CS_PIN);

void setup() {
  Serial.begin(115200);
  if (CAN_OK == CAN.begin(MCP_STDEXT, CAN_1000KBPS, MCP_8MHZ)) {
    Serial.println("CAN init OK!\n");
  } else {
    Serial.println("CAN init FAILED!\n");
    while (1)
      ;
  }

  // for (int i = 0; i < 50; i++) {  // for mc3001 autobaud
  //   char meaningless[2] = { 0x10, 0x01 };
  //   CAN.sendMsgBuf(0x562, 0, 2, meaningless);
  //   delay(10);
  // }

  // delay(100);
}

void loop() {


  char startmsg[2] = { 0x01, 0x00 };
  char resetnodemsg[2] = { 0x81, 0x00 };

  CAN.sendMsgBuf(0x000, 0, 2, resetnodemsg);
  delay(1000);

  while (CAN_MSGAVAIL == CAN.checkReceive()) {

    unsigned long canId = 0;
    unsigned char len = 0;
    unsigned char buf[16];

    CAN.readMsgBuf(&canId, &len, buf);

    Serial.print("ID: 0x");
    Serial.print(canId, HEX);
    Serial.print("  Data: ");

    for (int i = 0; i < len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  CAN.sendMsgBuf(0x000, 0, 2, startmsg);
  delay(100);
}