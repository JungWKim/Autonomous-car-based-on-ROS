#include <math.h>
#include <MsTimer2.h>
#include <SPI.h>

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

#define encoderL    2
#define encoderL_g  3
#define encoderR   21
#define encoderR_g 20

#define EA  13
#define A1  12
#define A2  11

#define B3  10
#define B4   9
#define EB   8

#define trig 37
#define echo 33

//   generate variables
//------------------------------------------------
boolean left_steering, right_steering;
int vel_L = 100, vel_R = 100;

const int ppr = 50;
volatile int pulseCountL = 0, pulseCountR = 0;
volatile int rpmL, rpmR;

volatile const float Kp = 1.1;

volatile int error, speed_gap;
volatile double Pcontrol, PIDcontrol;

float reflect_duration, obstacle_distance, velocity, ttc, rpm;
short serial_lock = 0;
int rxBuffer, target_gap;

//   interrupt function definitions
//------------------------------------------------
void speed_limit()
{
    if(vel_L > 250) vel_L = 250;
    else if(vel_L < 50) vel_L = 50;
        
    if(vel_R > 250) vel_R = 250;
    else if(vel_R < 50) vel_R = 50;
}


void speedCalibration()
{
  rpmL = int(pulseCountL / 0.5 / ppr) * 60;
  rpmR = int(pulseCountR / 0.5 / ppr) * 60;

  if(left_steering)
  {
    speed_gap = rpmR - rpmL;
    error = speed_gap - target_gap;
    Pcontrol = Kp * error;
    PIDcontrol = Pcontrol;
    if(speed_gap > target_gap)      vel_R += PIDcontrol;
    else if(speed_gap < target_gap) vel_R -= PIDcontrol;
    speed_limit();
    speedSetup(vel_L, vel_R);
  }
  else if(right_steering)
  {
    speed_gap = rpmL - rpmR;
    error = speed_gap - target_gap;
    Pcontrol = Kp * error;
    PIDcontrol = Pcontrol;
    if(speed_gap > target_gap)      vel_L += PIDcontrol;
    else if(speed_gap < target_gap) vel_L -= PIDcontrol;
    speed_limit();
    speedSetup(vel_L, vel_R);
  }
  else
  {
    if(rpmL != rpmR)
    {
      if(rpmL < rpmR) vel_L += 1;
      else            vel_R += 1;
      speed_limit();
      speedSetup(vel_L, vel_R);
    }
  }

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


//   function definitions
//------------------------------------------------
void speedSetup(int left, int right)
{
  analogWrite(EA, left);
  analogWrite(EB, right);
}

//direction set to move forward
void moveFront()
{
  digitalWrite(A2, LOW);
  digitalWrite(A1, HIGH);
  digitalWrite(B4, LOW);
  digitalWrite(B3, HIGH);
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

  moveFront();

  attachInterrupt(digitalPinToInterrupt(encoderL), pulseCounterL, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderR), pulseCounterR, RISING);
   
  MsTimer2::set(200, speedCalibration);
  MsTimer2::start();
  Serial.begin(57600);
  speedSetup(0, 0);//initial speed >> 0
}


//   Publish received data from Jetson TX2
//---------------------------------------------
void loop()
{    
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  reflect_duration = pulseIn(echo, HIGH);
  obstacle_distance = float(340 * reflect_duration) / 5000;
  rpm = (rpmL + rpmR) / 2;
  velocity = (rpm * 6.6 * 3.141592) / 60;
  ttc = (obstacle_distance / velocity) - 1;// 1 is system delay
  Serial.print("distance : ");
  Serial.println(obstacle_distance);
  Serial.print("ttc : ");
  Serial.println(ttc);

  if(ttc <= 2)
  {
    serial_lock = 1;
    speedSetup(0, 0);
  }
  else if(ttc <= 3)
  {
    serial_lock = 0;
    speedSetup(vel_L - 50, vel_R - 50);
  }
  else if(ttc <= 4)
  {
    serial_lock = 0;
    speedSetup(vel_L - 30, vel_R - 30);
  }
  else
  {
    serial_lock = 0;
    speedSetup(vel_L, vel_R);
  }
}


void serialEvent()
{
    if(!serial_lock)
    {
        while(Serial.available() > 0)
        {
            rxBuffer = Serial.read();
            target_gap = (signed int)rxBuffer;
            Serial.print("auto command : ");
            Serial.println(target_gap);
            if(target_gap > 0)
            {
                left_steering = true;
                right_steering = false;
            }
            else if(target_gap < 0)
            {
                left_steering = false;
                right_steering = true;
            }
            else
            {
                left_steering = false;
                right_steering = false;
                vel_L = 100;
                vel_R = 100;
            }
        }
    }
}
