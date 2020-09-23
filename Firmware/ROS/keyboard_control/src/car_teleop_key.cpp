#include <ros/ros.h>
#include <stdlib.h>

#include "keyboard_control/teleop_car.h"
#include "terminal_input.h"

#define FORWARD       1
#define BACKWARD      2
#define LEFTFORWARD   3
#define RIGHTFORWARD  4
#define LEFTBACKWARD  5
#define RIGHTBACKWARD 6
#define STOP          7
#define SPEEDUP       8
#define SPEEDDOWN     9

// main function
//-------------------------------------------------------------------------
int main(int argc, char **argv)
{
	int key = 0;

	init_keyboard();
	
	ros::init(argc, argv, "car_teleop_key");
	ros::NodeHandle nh;
	ros::Publisher ros_pub = nh.advertise<keyboard_control::teleop_car>("toArduino", 100);
	keyboard_control::teleop_car msg;

	//  27 equals ESC
	while(ros::ok() && key != 27)
	{
		if(kbhit())
		{
			switch(key = readch())
			{
				case 'w': 
					msg.data = FORWARD;
					ROS_INFO("send msg = %d", msg.data);
					ros_pub.publish(msg);
				      	break;
				case 'x': 
					msg.data = BACKWARD;
					ROS_INFO("send msg = %d", msg.data);
					ros_pub.publish(msg);
				      	break;
				case 'q': 
					msg.data= LEFTFORWARD;
					ROS_INFO("send msg = %d", msg.data);
					ros_pub.publish(msg);
				      	break;
				case 'e': 
					msg.data = RIGHTFORWARD;
					ROS_INFO("send msg = %d", msg.data);
					ros_pub.publish(msg);
				      	break;
				case 'z': 
					msg.data = LEFTBACKWARD;
					ROS_INFO("send msg = %d", msg.data);
					ros_pub.publish(msg);
				      	break;
				case 'c': 
					msg.data = RIGHTBACKWARD;
					ROS_INFO("send msg = %d", msg.data);
					ros_pub.publish(msg);
				      	break;
				case 's': 
					msg.data = STOP;
					ROS_INFO("send msg = %d", msg.data);
					ros_pub.publish(msg);
				      	break;
				case 'u': 
					msg.data = SPEEDUP;
					ROS_INFO("send msg = %d", msg.data);
					ros_pub.publish(msg);
				      	break;
				case 'j': 
					msg.data = SPEEDDOWN;
					ROS_INFO("send msg = %d", msg.data);
					ros_pub.publish(msg);
				      	break;
			}
		}
	}

	msg.data = STOP;
	ros_pub.publish(msg);
	ROS_INFO("Program Exiting..........");
	close_keyboard();

	return 0;
}

//reference site : https://m.blog.naver.com/PostView.nhn?blogId=tipsware&logNo=221009514492&proxyReferer=https:%2F%2Fwww.google.com%2F
//https://corsa.tistory.com/16
