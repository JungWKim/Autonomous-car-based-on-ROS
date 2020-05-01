#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16MultiArray.h>

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

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

unsigned short data[12];

void messageCb( const std_msgs::UInt16MultiArray& control_msg){
  analogWrite(enableA_forward, control_msg.data[0]);
  digitalWrite(insert1A_forward, control_msg.data[1]);
  digitalWrite(insert2A_forward, control_msg.data[2]);
  
  analogWrite(enableB_forward, control_msg.data[3]);
  digitalWrite(insert3B_forward, control_msg.data[4]);
  digitalWrite(insert4B_forward, control_msg.data[5]);

  analogWrite(enableA_backward, control_msg.data[6]);
  digitalWrite(insert1A_backward, control_msg.data[7]);
  digitalWrite(insert2A_backward, control_msg.data[8]);
  
  analogWrite(enableB_backward, control_msg.data[9]);
  digitalWrite(insert3B_backward, control_msg.data[10]);
  digitalWrite(insert4B_backward, control_msg.data[11]);

  for(int i = 0; i<12; i++)
    data[i] = control_msg.data[i];
}


ros::Subscriber<std_msgs::UInt16MultiArray> sub("motor_control", messageCb );


char message[50];


void status_analyze(unsigned short *data){
  
  //same speed(go or back straight)
  if(data[0] == data[3]) {
    if(data[1] > data[2])
      message[50] = "";
  }
  
  //turn left or right
  else {
    
  }
}


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



void loop()
{  
  status_analyze(data);
  
  str_msg.data = message;
  chatter.publish( &str_msg );
  
  nh.spinOnce();
  delay(10);
}
