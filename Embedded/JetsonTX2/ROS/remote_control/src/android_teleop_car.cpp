#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <ros/ros.h>

#include "remote_control/teleop_car.h"
 
#define BUF_SIZE 1
//#define BUF_SIZE 512

#define FORWARD       1
#define BACKWARD      2
#define LEFTFORWARD   3
#define RIGHTFORWARD  4
#define LEFTBACKWARD  5
#define RIGHTBACKWARD 6
#define STOP          7
#define SPEEDUP       8
#define SPEEDDOWN     9
#define EXIT         10
#define VOICE_START  11
#define VOICE_END    12
 
using namespace std;

void interrupt_handler(int sig)
{
    exit(0);
}
 
int main(int argc, char*argv[])
{
    int server_fd, client_fd;
    char buffer[1];
    struct sockaddr_in server_addr, client_addr;
    char temp[20];
    int msg_size;
    socklen_t len;
    char data;
    char thr;

    signal(SIGINT, interrupt_handler);

    ros::init(argc, argv, "android_teleop_car");
    ros::NodeHandle nh;
    ros::Publisher ros_pub = nh.advertise<remote_control::teleop_car>("toArduino", 100);
    remote_control::teleop_car msg;
 
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    { 
        ROS_INFO("Server : Can't open stream socket");
        exit(0);
    }

    //server_Addr 을 NULL로 초기화
    memset(&server_addr, 0x00, sizeof(server_addr));
 
    //server_addr 셋팅
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(9999);
 
    //server_fd소켓과 server_addr소켓구조체를 서로 연결
    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) <0)
    {
        ROS_INFO("Server : Can't bind local address.");
        exit(0);
    }
 
    //소켓을 수동 대기모드로 설정
    if (listen(server_fd, 5) < 0)
    {
        ROS_INFO("Server : Can't listening connect.");
        exit(0);
    }
    memset(buffer, 0x00, sizeof(buffer));

    len = sizeof(client_addr);
    ROS_INFO("Server : wating connection request.");

    client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &len);
    inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, temp, sizeof(temp));
    ROS_INFO("Server : %s client connected.", temp);

    while (1)
    {
	    client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &len);
  	    inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, temp, sizeof(temp));
	    recv(client_fd, buffer, BUF_SIZE, 0);

		while(1)
		{
            switch(buffer[0])
            {
                case 'W': msg.data = FORWARD;       break;
                case 'X': msg.data = BACKWARD;      break;
                case 'S': msg.data = STOP;          break;
                case 'Q': msg.data = LEFTFORWARD;   break;
                case 'E': msg.data = RIGHTFORWARD;  break;
                case 'Z': msg.data = LEFTBACKWARD;  break;
                case 'C': msg.data = RIGHTBACKWARD; break;
                case 'U': msg.data = SPEEDUP;       break;
                case 'J': msg.data = SPEEDDOWN;     break;
                case 'I': msg.data = STOP;          break;
                case 'V': ROS_INFO("voice control start");
                          break;
                case 'B': ROS_INFO("voice control end");
                          break;
            }
            if(buffer[0] == 'I')
            {
                ros_pub.publish(msg);
                ROS_INFO("buffer : %c", buffer[0]);
                ROS_INFO("send msg : %d", msg.data);
                memset(buffer, 0x00, sizeof(buffer));
                close(server_fd);
                ROS_INFO("Program exiting.....");
                return 0;
            }
            ros_pub.publish(msg);
            ROS_INFO("buffer : %c", buffer[0]);
            ROS_INFO("send msg : %d", msg.data);
            break;
		}
        memset(buffer, 0x00, sizeof(buffer));
    }
    close(server_fd);
    return 0;
}
