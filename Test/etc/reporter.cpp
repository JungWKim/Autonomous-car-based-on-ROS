#include <ros/ros.h>
#include <led_control/MsgLed.h>

void msgCallback(const led_control::MsgLed::ConstPtr& msg)
{
	if(msg->data == -1)
		ROS_INFO("wiringpi setup error");
	else if(msg->data == 1)
		ROS_INFO("Led status : Red Light[O], Green Light[X]");
	else
		ROS_INFO("Led status : Red Light[X], Green Light[O]");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reporter");
	ros::NodeHandle nh;
	ros::Subscriber ros_tutorial_sub = nh.subscribe("led_msg", 100, msgCallback);

	ros::spin();

	return 0;
}
