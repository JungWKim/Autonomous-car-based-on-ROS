#include <math.h>
#include <MsTimer2.h>
#include <SPI.h>
#include <CAN.h>

/*----------------
 * How to control motor
 * 
 * 1. 1A(HIGH) && 2A(LOW) >> reverse clock direction
 * 2. 1A(LOW) && 2A(HIGH) >> clock direction
 * 3. 3B(HIGH) && 4B(LOW) >> reverse clock direction
 * 4. 3B(LOW) && 4B(HIGH) >> clock direction
 * 
 */

//  Assigning pin numbers
//------------------------------------------------

#define encoderR   18
#define encoderL   21

#define EA  6
#define A1  12
#define A2  11

#define B3   7
#define B4   9
#define EB   8

#define trig 37
#define echo 33

//   generate variables
//------------------------------------------------
boolean left_steering, right_steering;
boolean steering_lock = false;
int vel_L = 70, vel_R = 70;

const int ppr = 1800;
const float system_delay = 0.5;
volatile int pulseCountL = 0, pulseCountR = 0;
volatile int rpmL, rpmR;
volatile int speed_adjust = 0;

volatile const float Kp = 10.0;

volatile int error, rpm_gap;
volatile double Pcontrol;

float reflect_duration, obstacle_distance, velocity, ttc, rpm;
float steering_angle;
int target_rpm_gap;
float rxBuffer;

//   interrupt function definitions
//------------------------------------------------
void speed_limit()
{
    if(vel_L > 250) vel_L = 250;
    else if(vel_L < 50) vel_L = 70;
        
    if(vel_R > 250) vel_R = 250;
    else if(vel_R < 50) vel_R = 70;
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
  _distance = ((float)(340 * reflect_duration) / 5000) / 2;
  return _distance;
}

float calculate_ttc()
{
  float _ttc;
  velocity = (rpm * 6.6 * 3.141592) / 60;
  _ttc = (obstacle_distance / velocity) - 1.0;// 1 is system delay
  if(_ttc < 0) _ttc = 0;
  return _ttc;
}

void aeb_handler()
{
  ttc = calculate_ttc();
  if(ttc <= (1.5 + system_delay))
  {
    steering_lock = true;
    speedSetup(0, 0);
    while(1)
    {
      obstacle_distance = detect_distance();
      if(obstacle_distance > 40) 
      {
        break;
      }
    }
    steering_lock = false;
  }
}

void print_status()
{
//  Serial.print("pulseCounterL : ");
//  Serial.println(pulseCountL);
//  Serial.print("pulseCounterR : ");
//  Serial.println(pulseCountR);
  Serial.print("rpmL : ");
  Serial.println(rpmL);
  Serial.print("rpmR : ");
  Serial.println(rpmR);
//  Serial.print("distance : ");
//  Serial.println(obstacle_distance);
//  Serial.print("velocity : ");
//  Serial.println(velocity);
//  Serial.print("left speed : ");
//  Serial.println(vel_L);
//  Serial.print("right speed : ");
//  Serial.println(vel_R);
//  Serial.print("ttc : ");
//  Serial.println(ttc);
  Serial.print("received angle : ");
  Serial.println(steering_angle);
}

void speedCalibration()
{
  rpmL = (int)((pulseCountL / ppr) * (60.0 / 0.4));
  rpmR = (int)((pulseCountR / ppr) * (60.0 / 0.4));
  rpm = (rpmL + rpmR) / 2;

  if(!steering_lock)
  {
    if(left_steering)
    {
      rpm_gap = rpmR - rpmL;
      if(rpm_gap > target_rpm_gap) speed_adjust -= 10;
      else if(rpm_gap < target_rpm_gap) speed_adjust += 10;
      speedSetup(vel_L, vel_R + speed_adjust);
    }
    else if(right_steering)
    {
      rpm_gap = rpmL - rpmR;
      if(target_rpm_gap > rpm_gap) speed_adjust += 10;
      else if(target_rpm_gap < rpm_gap) speed_adjust -= 10;
      speedSetup(vel_L + speed_adjust, vel_R);
    }
    else
    {
      speedSetup(vel_L, vel_R);
    }
  }

  pulseCountL = 0;
  pulseCountR = 0;
}

void pulseCounterL() { pulseCountL++; }
void pulseCounterR() { pulseCountR++; }


//   function definitions
//------------------------------------------------
void speedSetup(int left, int right)
{
  analogWrite(EA, right);
  analogWrite(EB, left);
}

//direction set to move forward
void moveForward()
{
  digitalWrite(A2, HIGH);
  digitalWrite(A1, LOW);
  digitalWrite(B4, HIGH);
  digitalWrite(B3, LOW);
}


//  1. Setup all pins as output
//  2. Push and pull topics
//----------------------------------------------
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

  moveForward();
  
  Serial.begin(57600);

  attachInterrupt(digitalPinToInterrupt(encoderL), pulseCounterL, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderR), pulseCounterR, RISING);
   
  MsTimer2::set(400, speedCalibration);
  MsTimer2::start();

  speedSetup(vel_L, vel_R);//initial speed >> 0
}


//   Publish received data from Jetson TX2
//---------------------------------------------
void loop()
{    
  obstacle_distance = detect_distance();
  if(obstacle_distance <= 40)
  {
    aeb_handler();
  }
  if(Serial.available() > 0)
  {
    rxBuffer = Serial.read();
    steering_angle = (float)rxBuffer;
    // -6 ~ 6 is defined as natural bias
    // over 30 is considered that lane detection is failed
    if(steering_angle > -6 and steering_angle < 6) steering_angle = 0;
    target_rpm_gap = (int)(steering_angle * Kp);
    if(target_rpm_gap > 0)
    {
      left_steering = true;
      right_steering = false;
      speed_adjust = 0;
    }
    else if(target_rpm_gap < 0)
    {
      left_steering = false;
      right_steering = true;
      speed_adjust = 0;
    }
    else
    {
      left_steering = false;
      right_steering = false;
    }
  }
  else
  {
    left_steering = false;
    right_steering = false;
  }
  print_status();
}
