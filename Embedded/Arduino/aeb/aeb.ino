#include <math.h>
#include <MsTimer2.h>

#define encoderL 18
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

void speedSetup(int left, int right)
{
    analogWrite(EA, left);
    analogWrite(EB, right);
}

void speedCalibration()
{
    rpmL = (int)((pulseCountL / ppr) * (60.0 / 0.5));
    rpmR = (int)((pulseCountR / ppr) * (60.0 / 0.5));

//    error = rpmL - rpmR;
//    Pcontrol = Kp * abs(error);
//    if(error < 0) vel_L += Pcontrol;
//    else if(error > 0) vel_R += Pcontrol;
//    speed_limit();
//    speedSetup(vel_L, vel_R);
    
//    Serial.print("Left rpm: ");
//    Serial.println(rpmL);
//    Serial.print("Right rpm: ");
//    Serial.println(rpmR);
//    Serial.print("left speed : ");
//    Serial.println(vel_L);
//    Serial.print("right speed : ");
//    Serial.println(vel_R);
    
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

    digitalWrite(A2, HIGH);
    digitalWrite(A1, LOW);
    digitalWrite(B4, HIGH);
    digitalWrite(B3, LOW);
}

void loop()
{
    //speedSetup(170, 170);//initial speed
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    reflect_duration = pulseIn(echo, HIGH);
    obstacle_distance = ((float)(340 * reflect_duration) / 10000) / 2;
    rpm = (rpmL + rpmR) / 2;
    velocity = (rpm * 6.6 * 3.141592) / 60;
    ttc = (obstacle_distance / velocity) + 1.0;
    while(obstacle_distance <= 30)
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
      ttc = (obstacle_distance / velocity) + 1.0;
      if(ttc <= 3)
      {
          speedSetup(0, 0);
      }
      else if(ttc <= 4)
      {
          speedSetup(50, 50);
      }
      else if(ttc <= 5)
      {
          speedSetup(80, 80);
      }
      else
      {
        break;
      }
      Serial.print("distance : ");
      Serial.println(obstacle_distance);
      Serial.print("velocity : ");
      Serial.println(velocity);
      Serial.print("ttc : ");
      Serial.println(ttc);
    }
    speedSetup(170, 170);
//    else if(ttc <= 5)
//    {
//        speedSetup(70, 70);
//    }

    Serial.print("distance : ");
    Serial.println(obstacle_distance);
//    Serial.print("rpmL : ");
//    Serial.println(rpmL);
//    Serial.print("rpmR : ");
//    Serial.println(rpmR);
      Serial.print("velocity : ");
      Serial.println(velocity);
      Serial.print("ttc : ");
      Serial.println(ttc);
}
