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

int read_port()
{
    struct can_frame recvFrame;
    int recv_bytes = 0;

    key_t key=12345;
    int msqid;
    struct message msg;

    msg.msg_type = 1;
    if((msqid = msgget(key, IPC_CREAT|0666)) == -1){
        perror("send msgget");
        exit(0);
    }

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
            msg.data = (int)recvFrame.data[i];
            if(msgsnd(msqid, &msg, sizeof(int), 0) == -1)
            {
                perror("queue send");
                exit(0);
            }
        }
    }
}

int write_port()
{
    int n;
	struct can_frame sendFrame;

    key_t key=54321;
    int msqid;
    struct message msg;

    if((msqid = msgget(key, IPC_CREAT|0666)) == -1){
        perror("receiver msgget");
        exit(0);
    }


	sendFrame.can_id = 0x555;
	sendFrame.can_dlc = 1;

    while(1)
    {
        if(msgrcv(msqid, &msg, sizeof(int), 0, 0) == -1)
        {
            perror("queue receive");
            exit(0);
        }

	    sprintf(sendFrame.data, (char)msg.data);
    
        //이후 메시지 큐를 지운다.
        if(msgctl(msqid,IPC_RMID,NULL)==-1)
        {
            perror("msgctl failed");
            exit(0);
        }

	    if (write(soc, &sendFrame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
    		perror("Write");
    		return 1;
    	}
    }
}

void ctrl_C(int sig)
{
    if(close(soc) < 0)
    {
        perror("CAN Close");
        exit(0);
    }
}

int main()
{
    signal(SIGINT, ctrl_C);

    open_port();

    std::thread t1(read_port);
    std::thread t2(write_port);

    t1.join();
    t2.join();
}
