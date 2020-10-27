#include <math.h>
#include <MsTimer2.h>

#define encoderL 10
#define encoderR 21

#define EA 6
#define A1

#define trig 37
#define echo 33

float duration, distance;

void setup()
{
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    Serial.begin(57600);
}

void loop()
{
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    duration = pulseIn(echo, HIGH);
    distance = ((float)(340 * duration) / 10000) / 2;
    Serial.print("distance :");
    Serial.println(distance);
}
