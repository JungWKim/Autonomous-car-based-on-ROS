#include <mcp_can.h>
#include <SPI.h>

#define INT 53

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[1];

MCP_CAN CAN0(45);                               // Set CS to pin 10


void setup()
{
  Serial.begin(57600);
  CAN0.begin(CAN_500KBPS);                       // init can bus : baudrate = 500k 
  pinMode(INT, INPUT);                            // Setting pin 2 for /INT input
  Serial.println("MCP2515 Library Receive Example...");
}

void loop()
{
    if(!digitalRead(INT))                         // If pin 2 is low, read receive buffer
    {
      CAN0.readMsgBuf(&len, rxBuf);              // Read data: len = data length, buf = data byte(s)
      rxId = CAN0.getCanId();                    // Get message ID
      Serial.print("ID: ");
      Serial.print(rxId, HEX);
      Serial.print("  Data: ");
      for(int i = 0; i<len; i++)                // Print each byte of the data
      {
        if(rxBuf[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
        {
          Serial.print("0");
        }
        Serial.println(rxBuf[i]);
      }
      Serial.println();
    }
}
