#include "can.h"

int soc;
int read_can_port;

struct ifreq ifr;
struct sockaddr_can addr;

int open_port()
{
    /* open socket */
    soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    cout << "soc : " << soc << endl;
    if(soc < 0)
    {
        perror("[failed] Socket");
        return (-1);
    }
    cout << "[success] can socket created" << endl;

    strcpy(ifr.ifr_name, "can0");

    if (ioctl(soc, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("[failed] ioctl");
        return (-1);
    }
    cout << "[success] ioctl initialized" << endl;

	memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    cout << "[success] structure initialized" << endl;

    //fcntl(soc, F_SETFL, O_NONBLOCK);

    if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
		perror("[failed] CAN Bind");
        return (-1);
    }
    cout << "[success] can bind" << endl;
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
    int data = -70;

	sendFrame.can_id = 0x555;
	sendFrame.can_dlc = 1;
    for(int i = 0; i<1; i++)
        sendFrame.data[i] = (char)data;

    cout << "sending data...." << endl;
    while(1)
    {
	    if ((n = sendto(soc, &sendFrame, sizeof(sendFrame), 0, (struct sockaddr *)&addr, sizeof(addr))) < 0) {
	    //if ((n = write(soc, &sendFrame, sizeof(sendFrame))) < 0) {
	    //if (write(soc, &sendFrame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
    		perror("[failed] Write");
    		return 1;
    	}
    }
}

void ctrl_c(int sig)
{
    close(soc);
}

int main()
{
    signal(SIGINT, ctrl_c);
    open_port();
    write_port();
    if(close(soc) < 0)
    {
        perror("[failed] can socket close");
        return 1;
    }
    //std::thread t1(read_port);
    //std::thread t2(write_port);

    //t1.join();
    //t2.join();
}
