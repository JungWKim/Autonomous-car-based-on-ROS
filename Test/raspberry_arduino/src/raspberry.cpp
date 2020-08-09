#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "raspberry_arduino/fromRaspberry.h"

int buffer = 0;

void msgCallback(const std_msgs::Int32::ConstPtr& msg)
{
	buffer = msg->data;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "raspberry");
	ros::NodeHandle nh;
	ros::Publisher raspberry_pub = nh.advertise<raspberry_arduino::fromRaspberry>("toArduino", 100);
	ros::Subscriber raspberry_sub = nh.subscribe("toRaspberry", 100, msgCallback);
	raspberry_arduino::fromRaspberry msg;

	while(ros::ok())
	{
		msg.raspberry2arduino = buffer;
		ROS_INFO("%d", msg.raspberry2arduino);
		raspberry_pub.publish(msg);
	}

	ros::spin();

	return 0;
}
