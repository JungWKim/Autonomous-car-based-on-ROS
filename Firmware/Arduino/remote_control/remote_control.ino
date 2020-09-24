#include <ros.h>
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

#define hall_A_g
#define hall_A_y

#define EA  13
#define A1  12
#define A2  11

#define hall_B_g
#define hall_B_y

#define EB   8
#define B3  10
#define B4   9


//   Basic declaration to use rosserial
//------------------------------------------------
ros::NodeHandle  nh;
int buf, vel_L=100, vel_R=150;


void speedSetup(int left, int right)
{
  analogWrite(EA, left);
  analogWrite(EB, right);
}



void Forward()
{
  digitalWrite(A2, HIGH);
  digitalWrite(A1, LOW);

  digitalWrite(B4, LOW);
  digitalWrite(B3, HIGH);
}


void Backward()
{
  digitalWrite(A2, LOW);
  digitalWrite(A1, HIGH);

  digitalWrite(B4, HIGH);
  digitalWrite(B3, LOW);
}


void LeftForward()
{
  Forward();
  speedSetup(vel_L, vel_R + 80);
}


void RightForward()
{
  Forward();
  speedSetup(vel_L + 80, vel_R);
}


void LeftBackward()
{
  Backward();
  speedSetup(vel_L, vel_R + 80);
}


void RightBackward()
{
  Backward();
  speedSetup(vel_L + 80, vel_R);
}


void Stop()
{
  speedSetup(0, 0);
}


//to keep the car's moving direction, get the previous driving method as a parameter
void SpeedUp(int past_key)
{
  vel_L += 10;
  vel_R += 10;
  
  if(vel_L > 200) vel_L = 200;
  if(vel_R > 200) vel_R = 200;

  switch(past_key)
  {
    case 1:
    case 2: speedSetup(vel_L, vel_R); break;
    case 3:
    case 4: speedSetup(vel_L, vel_R + 80); break;
    case 5:
    case 6: speedSetup(vel_L + 80, vel_R); break;
    case 7: speedSetup(0, 0); break;
  }
}


//to keep the car's moving direction, get the previous driving method as a parameter
void SpeedDown(int past_key)
{
  vel_L -= 10;
  vel_R -= 10;
  
  if(vel_L < 80)  vel_L = 80;
  if(vel_R < 80) vel_R = 80;
  
  switch(past_key)
  {
    case 1:
    case 2: speedSetup(vel_L, vel_R); break;
    case 3:
    case 4: speedSetup(vel_L, vel_R + 80); break;
    case 5:
    case 6: speedSetup(vel_L + 80, vel_R); break;
    case 7: speedSetup(0, 0); break;
  }
}



//   Subscriber function
//------------------------------------------------
void messageCb(const std_msgs::Int32& msg) {

  switch(msg.data){
    case 1: Forward();       speedSetup(vel_L, vel_R); buf = msg.data; break;
    case 2: Backward();      speedSetup(vel_L, vel_R); buf = msg.data;  break;
    case 3: LeftForward();   buf = msg.data; break;
    case 4: RightForward();  buf = msg.data; break;
    case 5: LeftBackward();  buf = msg.data; break;
    case 6: RightBackward(); buf = msg.data; break;
    case 7: Stop();          buf = msg.data; break;
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

  //pinMode(EA, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A1, OUTPUT);

  //pinMode(EB, OUTPUT);
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
