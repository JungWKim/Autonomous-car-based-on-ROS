#define encoderYellow 2
#define encoderGreen 3

int countYellow = 0;
int countGreen = 0;

void readEncoderYellow()
{
    countYellow++;
    Serial.print("Y:");
    Serial.println(countYellow);
}

void readEncoderGreen()
{
    countGreen++;
    Serial.print("G:");
    Serial.println(countGreen);
}

void setup()
{
    Serial.begin(57600);
    pinMode(encoderYellow, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderYellow), readEncoderYellow, RISING);
    pinMode(encoderGreen, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderGreen), readEncoderGreen, RISING);
}

void loop()
{

}
