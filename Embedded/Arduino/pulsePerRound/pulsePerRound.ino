#define encoderYellow 21
#define encoderGreen 20

#define EA 8
#define A1 9
#define A2 10

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
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(encoderYellow), readEncoderYellow, RISING);

    digitalWrite(A1, HIGH);
    digitalWrite(A2, LOW);
    analogWrite(EA, 60);
}

void loop()
{

}
