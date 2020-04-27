#include <ros/ros.h>
#include <wiringPi.h>

#define red 8

int main(int argc, char **argv)
{
	ros::init(argc, argv, "led_test");
	ros::NodeHandle nh;

	if(wiringPiSetup() == -1)
	{
		ROS_INFO("wiringPI setup error");
		return 0;
	}

	pinMode(red, OUTPUT);

	while(ros::ok())
	{
		digitalWrite(red, HIGH);
		ROS_INFO("red light on");
		ros::Duration(1.0).sleep();

		digitalWrite(red, LOW);
		ROS_INFO("red light off");
		ros::Duration(1.0).sleep();
	}

	return 0;
}
