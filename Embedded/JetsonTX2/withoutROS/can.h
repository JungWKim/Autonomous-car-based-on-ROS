#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include <iostream>
#include <thread>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/msg.h>

#include <linux/can.h>
#include <linux/can/raw.h>

using namespace std;

struct message{
        long msg_type;
        int data;
};
