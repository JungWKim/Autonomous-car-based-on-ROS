#include <ros/ros.h>
#include <wiringPi.h>
#include <led_control/MsgLed.h>

#define red 8
#define green 9

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	ros::Publisher ros_tutorial_pub = nh.advertise<led_control::MsgLed>("led_msg", 100);
	ros::Rate loop_rate(10);

	led_control::MsgLed msg;
	int count = 0;

	if(wiringPiSetup() == -1)
	{
		msg.data = -1;
		ros_tutorial_pub.publish(msg);
		return 0;
	}

	pinMode(red, OUTPUT);
	pinMode(green, OUTPUT);

	while(ros::ok())
	{
		digitalWrite(red, 1);
		digitalWrite(green, 0);
		msg.data = 1;
		ros_tutorial_pub.publish(msg);
		delay(1000);

		digitalWrite(red, 0);
		digitalWrite(green, 1);
		msg.data = 2;
		ros_tutorial_pub.publish(msg);
		delay(1000);

		loop_rate.sleep();

		count++;
	}

	return 0;
}
