#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16MultiArray.h>


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
#define EA_R   0
#define A1_R  53
#define A2_R  51

#define EB_R  10
#define B3_R  43
#define B4_R  41

#define EA_L   8
#define A1_L  22
#define A2_L  24

#define EB_L   3
#define B3_L   4
#define B4_L   5

#define SPEED 40


//   Basic declaration to use rosserial
//------------------------------------------------
ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

// array to store data from publisher
unsigned short data[6];


//   Motor control part
//------------------------------------------------
void messageCb( const std_msgs::UInt16MultiArray& control_msg){

  analogWrite(EA_L, SPEED);
  analogWrite(EA_R, SPEED);
  analogWrite(EB_R, SPEED);
  analogWrite(EB_L, SPEED);

#if 1

  digitalWrite(A1_L, control_msg.data[1]);
  digitalWrite(A2_L, control_msg.data[2]);
  
#endif
  
#if 1

  digitalWrite(B3_L, control_msg.data[1]);
  digitalWrite(B4_L, control_msg.data[2]);
  
#endif

#if 1

  digitalWrite(A1_R, control_msg.data[4]);
  digitalWrite(A2_R, control_msg.data[5]);

#endif

#if 1

  digitalWrite(B3_R, control_msg.data[4]);
  digitalWrite(B4_R, control_msg.data[5]);
  
#endif
  
  for(int i = 0; i<6; i++)
    data[i] = control_msg.data[i];
}


ros::Subscriber<std_msgs::UInt16MultiArray> sub("motor_control", messageCb );


//  message for publishing topic
char message[50];


//   check out the status of the vehicle
//----------------------------------------------------
void status_analyze(unsigned short *data){
#if 0
  //same speed(go or back straight)
  if(data[0] == data[3]) {
    if(data[1] == 1 && data[2] == 1)
     strcpy(message, "[Forward] [L: %d, 1, 0] [R: %d, 1, 0]", data[0], data[3]);
    else if(data[1] == 1 && data[2] == 0)
     strcpy(message, "[Rotation Right] [L: %d, 1, 0] [R: %d, 0, 1]", data[0], data[3]);
    else if(data[1] == 0 && data[2] == 1)
     strcpy(message, "[Rotation Left] [L: %d, 0, 1] [R: %d, 1, 0]", data[0], data[3]);
    else if(data[1] == 0 && data[2] == 0)
     strcpy(message, "[Rotation Left] [L: %d, 0, 1] [R: %d, 0, 1]", data[0], data[3]);
  }
  
  //turn left or right
  else if(data[0] > data[3]) {
    if(data[1] == 1 && data[2] == 1)
     strcpy(message, "[Turn Right] [L: %d, 1, 0] [R: %d, 1, 0]", data[0], data[3]);
    else if(data[1] == 0 && data[2] == 0)
     strcpy(message, "[Right back] [L: %d, 0, 1] [R: %d, 0, 1]", data[0], data[3]);
  }
  
  else if(data[0] < data[3]) {
    if(data[1] == 1 && data[2] == 1)
     strcpy(message, "[Turn Left] [L: %d, 1, 0] [R: %d, 1, 0]", data[0], data[3]);
    else if(data[1] == 0 && data[2] == 0)
     strcpy(message, "[Left back] [L: %d, 0, 1] [R: %d, 0, 1]", data[0], data[3]);
  }
#endif
}

//  1. Setup all pins as output
//  2. Push and pull topics
//----------------------------------------------
void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);

  pinMode(A1_L, OUTPUT);
  pinMode(A2_L, OUTPUT);

  pinMode(B3_L, OUTPUT);
  pinMode(B4_L, OUTPUT);
  
  pinMode(A1_R, OUTPUT);
  pinMode(A2_R, OUTPUT);

  pinMode(B3_R, OUTPUT);
  pinMode(B4_R, OUTPUT);
}


//   Publish status information According to the status_analyze()
//---------------------------------------------
void loop()
{  
  status_analyze(data);
  
  str_msg.data = message;
  chatter.publish( &str_msg );
  
  nh.spinOnce();
  delay(10);
}
