#include <ros/ros.h>
#include <wiringPi.h>
#include <softPwm.h>

#define enableB 8  //speed control
#define insert3B 9
#define insert4B 7

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;

	if(wiringPiSetup() == -1)
	{
		ROS_INFO("wiringpi setup error");
		return 0;
	}

	pinMode(enableB, OUTPUT);
	pinMode(insert3B, OUTPUT);
	pinMode(insert4B, OUTPUT);

	softPwmCreate(enableB, 0, 100);
	softPwmWrite(enableB, 40);

	ROS_INFO("motor move!!!");

	while(ros::ok())
	{
		digitalWrite(insert3B, HIGH);
		digitalWrite(insert4B, LOW);
		ROS_INFO("[INFO] insert3B : HIGH  /  insert4B : LOW");
		delay(5000);

		digitalWrite(insert3B, LOW);
		digitalWrite(insert4B, HIGH);
		ROS_INFO("[INFO] insert3B : LOW  /  insert4B : HIGH");
		delay(5000);
	}

	return 0;
}
