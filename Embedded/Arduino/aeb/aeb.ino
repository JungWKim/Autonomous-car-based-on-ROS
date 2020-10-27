#include <math.h>
#include <MsTimer2.h>

#define encoderL 10
#define encoderR 21

#define EA 6
#define A1 12
#define A2 11

#define B3 7
#define B4 9
#define EB 8

#define trig 37
#define echo 33

float reflect_duration, obstacle_distance, velocity, ttc;
int vel_L = 170, vel_R = 170;
const float ppr = 1800;
volatile float pulseCountL = 0, pulseCountR = 0;
volatile int rpmL, rpmR, rpm;

const float Kp = 10.0;
volatile int error;

volatile double Pcontrol;

void speed_limit()
{
   if(vel_L > 250) vel_L = 250;
   else if(vel_L < 50) vel_L = 50;
              
   if(vel_R > 250) vel_R = 250;
   else if(vel_R < 50) vel_R = 50;
                          
}

void speedSetup(int left, int right)
{
    analogWrite(EA, left);
    analogWrite(EB, right);
}

void speedCalibration()
{
    rpmL = (int)((pulseCountL / ppr) * (60.0 / 0.5));
    rpmR = (int)((pulseCountR / ppr) * (60.0 / 0.5));

    error = rpmL - rpmR;
    Pcontrol = Kp * abs(error);
    if(error < 0) vel_L += Pcontrol;
    else if(error > 0) vel_R += Pcontrol;
    speed_limit();
    speedSetup(vel_L, vel_R);
    
    Serial.print("Left rpm: ");
    Serial.println(rpmL);
    Serial.print("Right rpm: ");
    Serial.println(rpmR);
    Serial.print("left speed : ");
    Serial.println(vel_L);
    Serial.print("right speed : ");
    Serial.println(vel_R);
    
    pulseCountL = 0;
    pulseCountR = 0;
}

void pulseCounterL() { pulseCountL++; }
void pulseCounterR() { pulseCountR++; }

void setup()
{
    pinMode(encoderL, INPUT);
    pinMode(EA, OUTPUT);
    pinMode(A2, OUTPUT);
    pinMode(A1, OUTPUT);
               
    pinMode(encoderR, INPUT);
    pinMode(EB, OUTPUT);
    pinMode(B3, OUTPUT);
    pinMode(B4, OUTPUT);

    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    Serial.begin(57600);
    
    attachInterrupt(digitalPinToInterrupt(encoderL), pulseCounterL, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderR), pulseCounterR, RISING);
           
    MsTimer2::set(500, speedCalibration);
    MsTimer2::start();
    speedSetup(170, 170);//initial speed

    digitalWrite(A2, LOW);
    digitalWrite(A1, HIGH);
    digitalWrite(B4, LOW);
    digitalWrite(B3, HIGH);
}

void loop()
{
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    reflect_duration = pulseIn(echo, HIGH);
    obstacle_distance = ((float)(340 * reflect_duration) / 10000) / 2;
    rpm = (rpmL + rpmR) / 2;
    velocity = (rpm * 6.6 * 3.141592) / 60;
    if(velocity == 0)
    {
        ttc = 1.0;
    }
    else
    {
        ttc = (obstacle_distance / velocity) + 1.0;
    }
    
    if(ttc <= 2.0)
    {
        vel_L = 0;
        vel_R = 0;
        speedSetup(vel_L, vel_R);
    }
    else if(ttc <= 3.0)
    {
        vel_L = 50;
        vel_R = 50;
        speedSetup(vel_L, vel_R);
    }
    else if(ttc <= 4.0)
    {
        vel_L = 70;
        vel_R = 70;
        speedSetup(vel_L, vel_R);
    }
    else if(ttc <= 5.0)
    {
        vel_L = 100;
        vel_R = 100;
        speedSetup(vel_L, vel_R);
    }
    else if(ttc <= 6.0)
    {
        vel_L = 140;
        vel_R = 140;
        speedSetup(vel_L, vel_R);
    }
    else
    {
        vel_L = 170;
        vel_R = 170;
        speedSetup(vel_L, vel_R);
    }
    Serial.print("distance :");
    Serial.println(obstacle_distance);
}
