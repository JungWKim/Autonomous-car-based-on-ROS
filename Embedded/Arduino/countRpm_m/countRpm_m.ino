#include <MsTimer2.h>

#define encoderYellow 21

#define EB 8
#define b1 7
#define b2 9

//#define EB 6
//#define b1 12
//#define b2 11

volatile float pulseCount = 0;
volatile float ppr = 1800;
volatile int velocity = 40, Tc = 500;
volatile int rpm_m;

void pulseCounter()
{
    pulseCount++;
}

void M()
{
    rpm_m = (int)((pulseCount / ppr) * (60.0 / 0.5));
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
    
    MsTimer2::set(Tc, M);
    MsTimer2::start();
    Serial.begin(57600);
}

void loop()
{
  
}
