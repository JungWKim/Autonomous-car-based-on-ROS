char left_or_right;
int rxdata;

void setup()
{
    Serial.begin(57600);
}

void loop()
{

}

void serialEvent()
{
  while(Serial.available() > 0)
  {
    left_or_right = Serial.read();
    rxdata = (signed int)left_or_right;
    Serial.print("pid : ");
    Serial.println(rxdata);
  }
}
