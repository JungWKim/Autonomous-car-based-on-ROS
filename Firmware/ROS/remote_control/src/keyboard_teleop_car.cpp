#include <ros/ros.h>
#include <stdlib.h>

#include "remote_control/teleop_car.h"
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
	
	ros::init(argc, argv, "keyboard_teleop_car");
	ros::NodeHandle nh;
	ros::Publisher ros_pub = nh.advertise<remote_control::teleop_car>("toArduino", 100);
	remote_control::teleop_car msg;

	//  27 equals ESC
	while(ros::ok() && key != 27)
	{
		if(kbhit())
		{
			switch(key = readch())
			{
				case 'w': msg.data = FORWARD;       break;
				case 'x': msg.data = BACKWARD;      break;
				case 'q': msg.data= LEFTFORWARD;    break;
				case 'e': msg.data = RIGHTFORWARD;  break;
				case 'z': msg.data = LEFTBACKWARD;  break;
				case 'c': msg.data = RIGHTBACKWARD; break;
				case 's': msg.data = STOP;          break;
				case 'u': msg.data = SPEEDUP;       break;
				case 'j': msg.data = SPEEDDOWN;     break;
			}
			ROS_INFO("send msg = %d", msg.data);
			ros_pub.publish(msg);
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
