#include <ros.h>
#include <math.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <MsTimer2.h>

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

#define encoderL   18
#define encoderL_g 19
#define encoderR   21
#define encoderR_g 20

#define EA  6
#define A1  12
#define A2  11

#define B3   7
#define B4   9
#define EB   8

#define trig 37
#define echo 33

//   Basic declaration to use rosserial
//------------------------------------------------
ros::NodeHandle  nh;
std_msgs::Int32MultiArray status_msg;


//   generate variables
//------------------------------------------------
int past_key;/*buffer to store previous state*/ 
boolean left_steering, right_steering, dont_move;
int vel_L = 170, vel_R = 170;

const float ppr = 1800;
volatile float pulseCountL = 0, pulseCountR = 0;
volatile int rpmL, rpmR, rpm;

const float Kp = 10.0;
const float Kd = 0.8;

volatile int error, speed_gapL, speed_gapR;
volatile int target_gap = 18;

volatile double Pcontrol;

float reflect_duration, obstacle_distance, velocity, ttc;


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
    rpmL = (int)((pulseCountL / ppr) * (60.0 / 0.5));
    rpmR = (int)((pulseCountR / ppr) * (60.0 / 0.5));

    if(!dont_move)
    {
      if(left_steering)
      {
        speed_gapL = rpmR - rpmL;
        error = speed_gapL - target_gap;
        Pcontrol = Kp * error;
        if(speed_gapL > target_gap)      vel_R += Pcontrol;
        else if(speed_gapL < target_gap) vel_R -= Pcontrol;
        speed_limit();
        speedSetup(vel_L, vel_R);
      }
      else if(right_steering)
      {
        speed_gapR = rpmL - rpmR;
        error = speed_gapR - target_gap;
        Pcontrol = Kp * error;
        if(speed_gapR > target_gap)      vel_L += Pcontrol;
        else if(speed_gapR < target_gap) vel_L -= Pcontrol;
        speed_limit();
        speedSetup(vel_L, vel_R);
      }
      else
      {
        error = rpmL - rpmR;
        Pcontrol = Kp * abs(error);
        if(error < 0) vel_L += Pcontrol;
        else if(error > 0) vel_R += Pcontrol;
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

//direction set to move backward
void moveBack()
{
  digitalWrite(A2, HIGH);
  digitalWrite(A1, LOW);
  digitalWrite(B4, HIGH);
  digitalWrite(B3, LOW);
}


void vertical_drive(int current_key)
{
  if(past_key != current_key)
  {
    dont_move = false;
    left_steering = false;
    right_steering = false;
    vel_L = 170;
    vel_R = 170;
    speedSetup(vel_L, vel_R);
  }
}


void left_side_drive(int current_key)
{
  if(past_key != current_key)
  {
    dont_move = false;
    left_steering = true;
    right_steering = false;
    vel_L = 60;
    vel_R = 230;
    speedSetup(vel_L ,vel_R);
  }
}


void right_side_drive(int current_key)
{
  if(past_key != current_key)
  {
    dont_move = false;
    left_steering = false;
    right_steering = true;
    vel_L = 230;
    vel_R = 60;
    speedSetup(vel_L, vel_R);
  }
}


void stop_motor(int current_key)
{
  if(past_key != current_key)
  {
    int i;
    dont_move = true;
    left_steering = false;
    right_steering = false;
    for(i = min(vel_L, vel_R) ; i>0 ; i-=10)
    {
        speedSetup(i, i);
    }  
  } 
}


//to keep the car's moving direction, get the previous driving method as a parameter
void speedUp(int current_key)
{
  vel_L += 10;
  vel_R += 10;

  speed_limit();
  speedSetup(vel_L, vel_R);
}


//to keep the car's moving direction, get the previous driving method as a parameter
void speedDown(int current_key)
{
  vel_L -= 10;
  vel_R -= 10;

  speed_limit();
  speedSetup(vel_L, vel_R);
}


//   Subscriber handler
//------------------------------------------------
void messageCb(const std_msgs::Int32& msg) {
  switch(msg.data){
    case 1: moveFront(); vertical_drive(msg.data);   past_key = msg.data; break;
    case 2: moveBack();  vertical_drive(msg.data);   past_key = msg.data; break;
    case 3: moveFront(); left_side_drive(msg.data);  past_key = msg.data; break;
    case 4: moveFront(); right_side_drive(msg.data); past_key = msg.data; break;
    case 5: moveBack();  left_side_drive(msg.data);  past_key = msg.data; break;
    case 6: moveBack();  right_side_drive(msg.data); past_key = msg.data; break;
    case 7: stop_motor(msg.data); past_key = msg.data; break;
    case 8: speedUp(msg.data);    break;
    case 9: speedDown(msg.data);  break;
  } 
}


// declare publisher and subscriber
//----------------------------------------------
ros::Publisher chatter("chatter", &status_msg);
ros::Subscriber<std_msgs::Int32> sub("toArduino", messageCb);


//  1. Setup all pins as output
//  2. Push and pull topics
//----------------------------------------------
void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);

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
  Serial.begin(57600);
  speedSetup(0, 0);//initial speed >> 0
}


//   Publish received data from Jetson TX2
//---------------------------------------------
void loop()
{      
  status_msg.data[0] = vel_L;
  status_msg.data[1] = vel_R;
  status_msg.data[2] = rpmL;
  status_msg.data[3] = rpmR;
  chatter.publish(&status_msg);
  nh.spinOnce();
  delay(100);
}
