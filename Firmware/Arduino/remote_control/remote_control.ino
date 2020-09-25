#include <ros.h>
#include <stdio.h>
#include <std_msgs/Int32.h>

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
//-----------------------------------------------

#define encoderL_g 3
#define encoderL_y 2

#define encoderR_g 20
#define encoderR_y 21

#define EA  13
#define A1  12
#define A2  11

#define B3  10
#define B4   9
#define EB   8


//   Basic declaration to use rosserial
//------------------------------------------------
ros::NodeHandle  nh;
int buf, vel_L = 100, vel_R = 100;


void speedSetup(int left, int right)
{
  analogWrite(EA, left);
  analogWrite(EB, right);
}

//direction set to move forward
void moveFront(int past_key)
{
  digitalWrite(A2, HIGH);
  digitalWrite(A1, LOW);
  digitalWrite(B4, HIGH);
  digitalWrite(B3, LOW);
  if(past_key == 7)
  {
    speedSetup(vel_L, vel_R);
    delay(500);
  }
  buf = past_key;
}

//direction set to move backward
void moveBack(int past_key)
{
  digitalWrite(A2, LOW);
  digitalWrite(A1, HIGH);
  digitalWrite(B4, LOW);
  digitalWrite(B3, HIGH);
  if(past_key == 7)
  {
    speedSetup(vel_L, vel_R);
    delay(500);
  }
  buf = past_key;
}


void Forward()
{
  speedSetup(vel_L, vel_R);
}


void Backward()
{
  speedSetup(vel_L, vel_R);
}


void LeftForward()
{
  speedSetup(vel_L, int(vel_R + 80));
}


void RightForward()
{
  speedSetup(int(vel_L + 80), vel_R);
}


void LeftBackward()
{
  speedSetup(vel_L, int(vel_R + 80));
}


void RightBackward()
{
  speedSetup(int(vel_L + 80), vel_R);
}


void Stop(int past_key)
{
  speedSetup(0, 0);   
  buf = past_key;
}


//to keep the car's moving direction, get the previous driving method as a parameter
void SpeedUp(int past_key)
{
  vel_L += 5;
  vel_R += 5;

  if(vel_L > 255) vel_L = 255;
  if(vel_R > 255) vel_R = 255;
  
  switch(past_key)
  {
    case 1:
    case 2: speedSetup(vel_L, vel_R); break;
    case 3:
    case 4: speedSetup(vel_L, int(vel_R + 80)); break;
    case 5:
    case 6: speedSetup(int(vel_L + 80), vel_R); break;
    case 7: speedSetup(0, 0); break;
  }
}


//to keep the car's moving direction, get the previous driving method as a parameter
void SpeedDown(int past_key)
{
  vel_L -= 5;
  vel_R -= 5;

  if(vel_L < 40)  vel_L = 40;
  if(vel_R < 40) vel_R = 40;
  
  switch(past_key)
  {
    case 1:
    case 2: speedSetup(vel_L, vel_R); break;
    case 3:
    case 4: speedSetup(vel_L, int(vel_R + 80)); break;
    case 5:
    case 6: speedSetup(int(vel_L + 80), vel_R); break;
    case 7: speedSetup(0, 0); break;
  }
}



//   Subscriber function
//------------------------------------------------
void messageCb(const std_msgs::Int32& msg) {
  switch(msg.data){
    case 1: moveFront(msg.data); Forward();       break;
    case 2: moveBack(msg.data);  Backward();      break;
    case 3: moveFront(msg.data); LeftForward();   break;
    case 4: moveFront(msg.data); RightForward();  break;
    case 5: moveBack(msg.data);  LeftBackward();  break;
    case 6: moveBack(msg.data);  RightBackward(); break;
    case 7: Stop(msg.data);  break;
    case 8: SpeedUp(buf);    break;
    case 9: SpeedDown(buf);  break;
  }
}
ros::Subscriber<std_msgs::Int32> sub("toArduino", messageCb);


std_msgs::Int32 int_msg;

//     Publisher registration part
//----------------------------------------------
ros::Publisher chatter("chatter", &int_msg);



//  1. Setup all pins as output
//  2. Push and pull topics
//----------------------------------------------
void setup()
{
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);

  pinMode(EA, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A1, OUTPUT);

  pinMode(EB, OUTPUT);
  pinMode(B3, OUTPUT);
  pinMode(B4, OUTPUT);

  speedSetup(0, 0);
}


//   Publish received data from Raspberry pi
//---------------------------------------------
void loop()
{    
  int_msg.data = buf;
  chatter.publish(&int_msg);
  nh.spinOnce();
  delay(100);
}
