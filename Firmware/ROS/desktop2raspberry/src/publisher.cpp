#include <ros/ros.h>
#include "raspberry_arduino/Control.h"
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>


#define FORWARD  1
#define BACKWARD 2
#define LEFT     3
#define RIGHT    4
#define STOP     5

static struct termios initial_settings, new_settings;
static int peek_character = -1;

void init_keyboard();
void close_keyboard();
int kbhit();
int readch();


// main function
//-------------------------------------------------------------------------
int main(int argc, char **argv)
{
	int key = 0;

	init_keyboard();
	
	ros::init(argc, argv, "publisher");
	ros::NodeHandle nh;
	ros::Publisher ros_pub = nh.advertise<raspberry_arduino::Control>("toArduino", 100);
	raspberry_arduino::Control msg;

	while(ros::ok() && key != 'q')
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
				case 'a': 
					msg.data = LEFT;
					ROS_INFO("send msg = %d", msg.data);
					ros_pub.publish(msg);
				      	break;
				case 'd': 
					msg.data = RIGHT;
					ROS_INFO("send msg = %d", msg.data);
					ros_pub.publish(msg);
				      	break;
				case 's': 
					msg.data = STOP;
					ROS_INFO("send msg = %d", msg.data);
					ros_pub.publish(msg);
				      	break;
			}
		}
	}

	msg.data = STOP;
	ros_pub.publish(msg);
	close_keyboard();

	return 0;
}


void init_keyboard()
{

             tcgetattr(0, &initial_settings);

             new_settings = initial_settings;

             new_settings.c_lflag &= ~ICANON;

             new_settings.c_lflag &= ~ECHO;

             new_settings.c_lflag &= ~ISIG;

             new_settings.c_cc[VMIN] = 1;

             new_settings.c_cc[VTIME] = 0;

             tcsetattr(0, TCSANOW, &new_settings);

}

void close_keyboard()
{
             tcsetattr(0, TCSANOW, &initial_settings);
}

int kbhit()
{
             char ch;

             int nread;

             if (peek_character != -1) return 1;

             new_settings.c_cc[VMIN] = 0;

             tcsetattr(0, TCSANOW, &new_settings);

             nread = read(0, &ch, 1);

             new_settings.c_cc[VMIN] = 1;

             tcsetattr(0, TCSANOW, &new_settings);

             if (nread == 1)
             {
                          peek_character = ch;

                          return 1;

             }

             return 0;
}

int readch()
{
             char ch;

             if (peek_character != -1)
             {
                          ch = peek_character;

                          peek_character = -1;

                          return ch;
             }

             read(0, &ch, 1);

             return ch;
}

//reference site : https://m.blog.naver.com/PostView.nhn?blogId=tipsware&logNo=221009514492&proxyReferer=https:%2F%2Fwww.google.com%2F
//https://corsa.tistory.com/16
