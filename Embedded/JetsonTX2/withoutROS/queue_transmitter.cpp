#include "queue.h"

int msqid;
struct message msg;

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
    
    msg.msg_type = 1;
    msg.data = 1;
    if((msqid = msgget(key, IPC_CREAT | 0666)) == -1)
    {
        perror("transmitter mssget");
        return 1;
    }

    while(1)
    {
        if(msgsnd(msqid, &msg, sizeof(int), 0) == -1)
        {
            perror("queue send error");
            return 1;
        }
        cout << "sending data" << endl;
        sleep(1);
    }
    return 0;
}
