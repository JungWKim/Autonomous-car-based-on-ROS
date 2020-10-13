#include "can.h"

int soc;
int read_can_port;

struct ifreq ifr;
struct sockaddr_can addr;

int open_port()
{
    /* open socket */
    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(soc < 0)
    {
        perror("Socket");
        return (-1);
    }

	memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    strcpy(ifr.ifr_name, "can0");

    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0)
    {
        return (-1);
    }

    fcntl(soc, F_SETFL, O_NONBLOCK);

    if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
		perror("CAN Bind");
        return (-1);
    }
}

#if 0
int read_port()
{
    struct can_frame recvFrame;
    int recv_bytes = 0;

    while(1)
    {
        recv_bytes = read(soc, &recvFrame, sizeof(struct can_frame));
       	if (recv_bytes < 0) {
	    	perror("CAN Read");
		    return 1;
	    }
       	for (int i = 0; i < recvFrame.can_dlc; i++)
        {
		    printf("message from arduino : %c", recvFrame.data[i]);
        }
    }
}
#endif

int write_port()
{
    int n;
	struct can_frame sendFrame;

	sendFrame.can_id = 0x555;
	sendFrame.can_dlc = 1;

    while(1)
    {
	    if (write(soc, &sendFrame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
    		perror("Write");
    		return 1;
    	}
    }
}

void ctrl_c(int sig)
{

    if(close(soc) < 0)
    {
        perror("CAN Close");
    }
    exit(0);
}

int main()
{
    signal(SIGINT, ctrl_c);

    open_port();
    write_port();

    //std::thread t1(read_port);
    //std::thread t2(write_port);

    //t1.join();
    //t2.join();
}
