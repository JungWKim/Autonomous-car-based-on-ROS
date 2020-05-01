#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16MultiArray.h>

//  Assigning pin numbers
//-----------------------------------------------
#define enableA_forward   3
#define insert1A_forward  2
#define insert2A_forward  4

#define enableB_forward   5
#define insert3B_forward  7
#define insert4B_forward  8

#define enableA_backward  6
#define insert1A_backward 10
#define insert2A_backward 11

#define enableB_backward  9
#define insert3B_backward 12
#define insert4B_backward 13


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
  analogWrite(enableA_forward, control_msg.data[0]);
  digitalWrite(insert1A_forward, control_msg.data[1]);
  digitalWrite(insert2A_forward, control_msg.data[2]);
  
  analogWrite(enableB_forward, control_msg.data[3]);
  digitalWrite(insert3B_forward, control_msg.data[4]);
  digitalWrite(insert4B_forward, control_msg.data[5]);

  analogWrite(enableA_backward, control_msg.data[0]);
  digitalWrite(insert1A_backward, control_msg.data[1]);
  digitalWrite(insert2A_backward, control_msg.data[2]);
  
  analogWrite(enableB_backward, control_msg.data[3]);
  digitalWrite(insert3B_backward, control_msg.data[4]);
  digitalWrite(insert4B_backward, control_msg.data[5]);

  for(int i = 0; i<6; i++)
    data[i] = control_msg.data[i];
}


ros::Subscriber<std_msgs::UInt16MultiArray> sub("motor_control", messageCb );


//  message for publishing topic
char message[50];


//   check out the status of the vehicle
//----------------------------------------------------
void status_analyze(unsigned short *data){
  
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
}

//  1. Setup all pins as output
//  2. Push and pull topics
//----------------------------------------------
void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);

  pinMode(enableA_forward, OUTPUT);
  pinMode(insert1A_forward, OUTPUT);
  pinMode(insert2A_forward, OUTPUT);

  pinMode(enableB_forward, OUTPUT);
  pinMode(insert3B_forward, OUTPUT);
  pinMode(insert4B_forward, OUTPUT);

  pinMode(enableA_backward, OUTPUT);
  pinMode(insert1A_backward, OUTPUT);
  pinMode(insert2A_backward, OUTPUT);

  pinMode(enableB_backward, OUTPUT);
  pinMode(insert3B_backward, OUTPUT);
  pinMode(insert4B_backward, OUTPUT);
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
