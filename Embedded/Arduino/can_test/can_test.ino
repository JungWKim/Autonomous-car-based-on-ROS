#include <mcp_can.h>
#include <SPI.h>

#define INT 12

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[1];

MCP_CAN CAN(8);                               // Set CS to pin 10

void setup()
{
  Serial.begin(115200);
  while( CAN_OK != CAN.begin(CAN_500KBPS))
  {
      Serial.println("CAN bus init failed");
      delay(100);
  }
  pinMode(INT, INPUT);                            // Setting pin 2 for /INT input
  //attachInterrupt(digitalPinToInterrupt(2), CAN_int, FALLING);
  Serial.println("MCP2515 Library Receive Example...");
}

//void CAN_int()
//{
//    unsigned char len = 0;
//    unsigned char buf[8] = "0";
//
//    CAN.readMsgBuf(&len, buf);
//    unsigned long canId = CAN.getCanId();
//    Serial.print("\nData from ID : 0x");
//    Serial.println(canId, HEX);
//    for(int i = 0; i < len; i++)
//    {
//        Serial.print(buf[i]);
//        Serial.print("\t");
//    }
//}

void loop()
{
    if(!digitalRead(INT))                         // If pin 2 is low, read receive buffer
    {
      CAN.readMsgBuf(&len, rxBuf);              // Read data: len = data length, buf = data byte(s)
      rxId = CAN.getCanId();                    // Get message ID
      Serial.print("ID: ");
      Serial.print(rxId, HEX);
      Serial.print("  Data: ");
      Serial.println(rxBuf[0], HEX);
//      for(int i = 0; i<len; i++)                // Print each byte of the data
//      {
//        if(rxBuf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
//        {
//          Serial.print("0");
//        }
//        Serial.println(rxBuf[i]);
//      }
    }
}
