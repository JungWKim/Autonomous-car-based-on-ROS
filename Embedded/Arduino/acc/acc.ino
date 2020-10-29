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
const int max_vel = 250;
short aeb_signal = 0;
const float ppr = 1800;
volatile float pulseCountL = 0, pulseCountR = 0;
volatile int rpmL, rpmR, rpm;

const float Kp = 10.0;
volatile int error;

volatile double Pcontrol;

void print_status()
{
    Serial.print("distance : ");
    Serial.println(obstacle_distance);
//    Serial.print("rpmL : ");
//    Serial.println(rpmL);
//    Serial.print("rpmR : ");
//    Serial.println(rpmR);
//    Serial.print("velocity : ");
//    Serial.println(velocity);
    Serial.print("ttc : ");
    Serial.println(ttc);
}

float calculate_ttc()
{
  float _ttc;
  velocity = (rpm * 6.6 * 3.141592) / 60;
  _ttc = (obstacle_distance / velocity) - 1;
  if(_ttc < 0) _ttc = 0;
  return _ttc;
}

void speedSetup(int left, int right)
{
    analogWrite(EA, left);
    analogWrite(EB, right);
}

float detect_distance()
{
    float _distance;
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    reflect_duration = pulseIn(echo, HIGH);
    _distance = ((float)(340 * reflect_duration) / 10000) / 2;
    return _distance;
}

void aeb_handler()
{
    if(obstacle_distance <= 40)
    {
      aeb_signal = 1;
      if(ttc > 3.5)
      {
          vel_L = 130;
          vel_R = 130;
          speedSetup(vel_L, vel_R);
      }
      else if(ttc <= 3.5 and ttc > 2.5)
      {
          vel_L = 100;
          vel_R = 100;
          speedSetup(vel_L, vel_R);
      }
      else if(ttc <= 2.5 and ttc > 1.5)
      {
          vel_L = 50;
          vel_R = 50;
          speedSetup(vel_L, vel_R);
      }
      else
      {
          vel_L = 0;
          vel_R = 0;
          speedSetup(vel_L, vel_R);
          while(1)
          {
            obstacle_distance = detect_distance();
            if(obstacle_distance > 15) break;
          }
      }
      obstacle_distance = detect_distance();
      ttc = calculate_ttc();
    }
    else
    {
      aeb_signal = 0;
    }
}

void speedCalibration()
{
    rpmL = (int)((pulseCountL / ppr) * (60.0 / 0.5));
    rpmR = (int)((pulseCountR / ppr) * (60.0 / 0.5));
    rpm = (rpmL + rpmR) / 2;
    aeb_handler();

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
    
    attachInterrupt(digitalPinToInterrupt(encoderL), pulseCounterL, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderR), pulseCounterR, RISING);
           
    MsTimer2::set(500, speedCalibration);
    MsTimer2::start();
    speedSetup(170, 170);//initial speed

    digitalWrite(A2, HIGH);
    digitalWrite(A1, LOW);
    digitalWrite(B4, HIGH);
    digitalWrite(B3, LOW);

    Serial.begin(57600);
}

void loop()
{
    if(!aeb_signal)
    {
      obstacle_distance = detect_distance();
      if(obstacle_distance >= 100)
      {
        speedSetup(max_ 
      }
    }
    print_status();
}
