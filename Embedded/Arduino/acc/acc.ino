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

#define front_echo 41
#define front_trig 45

float reflect_duration, front_obstacle_distance;
float error;
int Pcontrol;
float velocity, ttc;
float system_delay = 0.5;
const float target_distance = 40;
const float max_vel = 250;
const float ppr = 1800;
const float Kp = 8.0;
volatile float pulseCountL = 0, pulseCountR = 0;
volatile int rpmL, rpmR, rpm;

void print_status()
{
//    Serial.print("distance : ");
//    Serial.println(front_obstacle_distance);
//    Serial.print("rpmL : ");
//    Serial.println(rpmL);
//    Serial.print("rpmR : ");
//    Serial.println(rpmR);
//    Serial.print("velocity : ");
//    Serial.println(velocity);
//    Serial.print("ttc : ");
//    Serial.println(ttc);
    Serial.print("speed : ");
    Serial.println(150 + Pcontrol);
}

void speedLimit()
{
  if(Pcontrol > 100) Pcontrol = 100;
  if(Pcontrol < -150) Pcontrol = -150;
}

float calculate_ttc()
{
  float _ttc;

  _ttc = (front_obstacle_distance / velocity) - 1;
  if(_ttc < 0) _ttc = 0;
  return _ttc;
}

void speedSetup(int left, int right)
{
    analogWrite(EA, left);
    analogWrite(EB, right);
}

void detect_distance()
{
    digitalWrite(front_trig, LOW);
    delayMicroseconds(2);
    digitalWrite(front_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(front_trig, LOW);
    reflect_duration = pulseIn(front_echo, HIGH);
    front_obstacle_distance = ((float)(340 * reflect_duration) / 10000) / 2;
}

void aeb_handler()
{
  ttc = calculate_ttc();
  if(ttc <= (1.5 + system_delay))
  {
    speedSetup(0, 0);
    while(1)
    {
      detect_distance();
      if(front_obstacle_distance > 20) 
      {
        break;
      }
    }
  }
}

void speedCalibration()
{
    rpmL = (int)((pulseCountL / ppr) * (60.0 / 0.5));
    rpmR = (int)((pulseCountR / ppr) * (60.0 / 0.5));
    rpm = (rpmL + rpmR) / 2;
    velocity = (rpm * 6.6 * 3.141592) / 60.0;
    
    pulseCountL = 0;
    pulseCountR = 0;
}

void pulseCounterL() { pulseCountL++; }
void pulseCounterR() { pulseCountR++; }

void setup()
{
    pinMode(encoderR, INPUT);
    pinMode(EA, OUTPUT);
    pinMode(A2, OUTPUT);
    pinMode(A1, OUTPUT);
               
    pinMode(encoderL, INPUT);
    pinMode(EB, OUTPUT);
    pinMode(B3, OUTPUT);
    pinMode(B4, OUTPUT);

    pinMode(front_trig, OUTPUT);
    pinMode(front_echo, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(encoderL), pulseCounterL, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderR), pulseCounterR, RISING);
           
    MsTimer2::set(500, speedCalibration);
    MsTimer2::start();

    digitalWrite(A2, LOW);
    digitalWrite(A1, HIGH);
    digitalWrite(B4, LOW);
    digitalWrite(B3, HIGH);

    Serial.begin(57600);

    speedSetup(150, 150);
}

void loop()
{
  detect_distance();
  error = front_obstacle_distance - target_distance;
  Pcontrol = (int)(Kp * error);
  speedLimit();
  speedSetup(150 + Pcontrol, 150 + Pcontrol);
  if(front_obstacle_distance <= 15)
  {
    speedSetup((max_vel / 25) * 9, (max_vel / 25) * 9);
    aeb_handler();
  }
  print_status();
}
