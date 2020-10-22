int left_or_right;
int control;

void setup()
{
    Serial.begin(57600);
}

void loop()
{
  if(Serial.available() > 0)
  {
      if((recvBuffer = Serial.read()))
      {
          //Serial.print("received data : ");
          Serial.print(sendBuffer[0]);
          //Serial.print(
          //Serial.write(sendBuffer[1]);
          //Serial.write(sendBuffer[);
      }
  }
}
