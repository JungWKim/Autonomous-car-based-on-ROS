#include <ros.h>
#include <std_msgs/String.h>

#define enableB 2
#define insert3B 3
#define insert4B 4

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char message1[50] = "[INFO] insert3B : HIGH / insert4B : LOW";
char message2[50] = "[INFO] insert3B : LOW / insert4B : HIGH";

byte speedDC = 255;

void setup()
{
  nh.initNode();
  nh.advertise(chatter);

  pinMode(enableB, OUTPUT);
  pinMode(insert3B, OUTPUT);
  pinMode(insert4B, OUTPUT);
}

void loop()
{
  digitalWrite(insert3B, HIGH);
  digitalWrite(insert4B, LOW);
  analogWrite(enableB, 20);
  
  str_msg.data = message1;
  chatter.publish( &str_msg );
  
  nh.spinOnce();
  delay(1000);
}
