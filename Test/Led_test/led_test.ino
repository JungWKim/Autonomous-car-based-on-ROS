#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String msg;
ros::Publisher chatter("chatter", &msg);

char message1[50] = "[INFO] Red light [O], Green light [X]";
char message2[50] = "[INFO] Red light [X], Green light [O]";

void setup() {
  // put your setup code here, to run once:
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);
  msg.data = message1;
  chatter.publish(&msg);
  delay(1000);

  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);
  msg.data = message2;
  chatter.publish(&msg);
  delay(1000);
  
  nh.spinOnce();
  delay(1);
}
