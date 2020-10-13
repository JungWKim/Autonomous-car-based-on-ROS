#include "queue.h"

int msqid;
struct message recv_msg;

void interrupt_handler(int sig)
{
    if(msgctl(msqid, IPC_RMID, 0) == -1)
    {
        perror("msgctl failed");
    }
    cout << "queue deleted" << endl;
    exit(0);
}

int main()
{
    key_t key = 1234;

    signal(SIGINT, interrupt_handler);
    
    //msg.msg_type = 1;
    if((msqid = msgget(key, IPC_CREAT | 0666)) == -1)
    {
        perror("receiver mssget");
        return 1;
    }

    while(1)
    {
        if(msgrcv(msqid, &recv_msg, sizeof(recv_msg.data), 1, 0) == -1)
        {
            if(errno == ENOMSG)
            {
                printf("no message\n");
                break;
            }
            perror("queue receive error");
            return 1;
        }

        cout << "received message : " << recv_msg.data << endl;
        sleep(1);
    }
    return 0;
}
