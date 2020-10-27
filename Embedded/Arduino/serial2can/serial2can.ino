#include <CAN.h>

char rxBuffer;

void setup()
{
    Serial.begin(9600);
    if(!CAN.begin(500E3))
    {
        Serial.println("starting CAN failed");
        while(1);
    }
}

void loop()
{
    while(Serial.available() > 0)
    {
        if(rxBuffer = Serial.read())
        {
          Serial.print("received data from serial : ");
          Serial.println((signed int)rxBuffer);
          CAN.beginPacket(0x123);
          CAN.write(rxBuffer);
          CAN.endPacket();
          Serial.print("send data to can port : ");
          Serial.println((signed int)rxBuffer);
        }
    }
}

void serialEvent()
{

}
