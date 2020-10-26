#include <MsTimer2.h>

/*
#define encoderYellow 2
#define encoderGreen 3

#define EA 13
#define a1 12
#define a2 11

volatile int pulseCount = 0;
volatile int ppr = 50;
volatile int velocity = 250, Tc = 500;
volatile int rpm_m;

void pulseCounter()
{
    pulseCount++;
}

void M()
{
    rpm_m = int(pulseCount / 0.5 / ppr) * 60;
    Serial.print("M: ");
    Serial.println(rpm_m);
    Serial.print("pulse count: ");
    Serial.println(pulseCount);
    pulseCount = 0;
}

void setup()
{
    int i = velocity;

    pinMode(encoderYellow, INPUT);
    pinMode(a1, OUTPUT);
    pinMode(a2, OUTPUT);

    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
    analogWrite(EA, velocity);
    
    attachInterrupt(digitalPinToInterrupt(encoderYellow), pulseCounter, RISING);
    
    MsTimer2::set(500, M);
    MsTimer2::start();
    Serial.begin(57600);
}

void loop()
{
    delay(10000);
    for(int i = velocity; i > 0; i-=10)
    {
        analogWrite(EA, i);
        delay(500);
    }
}
*/

#define encoderYellow 18

#define EB 13
#define b1 12
#define b2 11

//#define EB 8
//#define b1 9
//#define b2 10

volatile int pulseCount = 0;
volatile int ppr = 50;
volatile int velocity = 180, Tc = 500;
volatile int rpm_m;

void pulseCounter()
{
    pulseCount++;

}

void M()
{
    rpm_m = int(pulseCount / 0.5 / ppr) * 60;
    Serial.print("RPM: ");
    Serial.println(rpm_m);
    Serial.print("pulse count: ");
    Serial.println(pulseCount);
    pulseCount = 0;
}

void setup()
{
    pinMode(encoderYellow, INPUT);
    pinMode(b1, OUTPUT);
    pinMode(b2, OUTPUT);

    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
    analogWrite(EB, velocity);
    
    attachInterrupt(digitalPinToInterrupt(encoderYellow), pulseCounter, RISING);
    
    MsTimer2::set(500, M);
    MsTimer2::start();
    Serial.begin(57600);
}

void loop()
{
}
