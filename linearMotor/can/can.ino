#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
struct can_frame resetmsg;
struct can_frame startmsg;
MCP2515 mcp2515(53);  // 10 for UNO, 53 for MEGA
u8 c=0;


void setup() {
  Serial.begin(9600);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  //Serial.print("Wait a while for mc3001\n");
  //delay(5000);
  //mcp2515.setLoopbackMode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");

  startmsg.can_id = 0x000; 
  startmsg.can_dlc = 2;
  startmsg.data[0] = 0x01;
  startmsg.data[1] = 0x00;

  mcp2515.sendMessage(&startmsg);
}

void loop() {
  mcp2515.sendMessage(&startmsg);

  

  // resetmsg.can_id = 0x000; 
  // resetmsg.can_dlc = 2;
  // resetmsg.data[0] = 0x81;
  // resetmsg.data[1] = 0x00;


  // char startmsg[2] = { 0x01, 0x00 };
  // char resetnodemsg[2] = { 0x81, 0x00 };

  //byte err = mcp2515.sendMessage(&resetmsg);
  //Serial.println(err);



  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print(c++); // print ID
    Serial.print(" "); // print ID
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");
    
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i],HEX);
      Serial.print(" ");
    }

    Serial.println();      
  }

  delay(500);
  

}
