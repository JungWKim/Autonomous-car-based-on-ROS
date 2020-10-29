#include "queue.h"
#include "serial.h"

int fd, rdcnt;
int msqid;
int buffer;

key_t key = 54321;
struct message recv_msg;

int write2serial_port()
{
    if((msqid = msgget(key, IPC_CREAT | 0666)) == -1){
        perror("receiver msgget");
        return 1;
    }

    sleep(3);

    while(1)
    {
        if(msgrcv(msqid, &recv_msg, sizeof(struct message), 1, 0) == -1)
        {
            if(errno == ENOMSG)
            {
                printf("no message\n");
                break;
            }
            perror("queue receive");
            return 1;
        }
        cout << "received from queue : " << recv_msg.control << endl;

        buffer = recv_msg.control;
        if((rdcnt = write(fd, &buffer, sizeof(int))) < 0)
        {
            perror("[failed] write data to serial");
            return 1;
        }
        cout << "write to serial port : " << buffer << endl;
    }
}

void ctrl_c(int sig)
{
    //메시지 큐를 지운다.
    if(msgctl(msqid,IPC_RMID,NULL)==-1)
    {
        perror("msgctl failed");
    }
    exit(0);
}

int main(int argc, char **argv)
{
    signal(SIGINT, ctrl_c);

    int baud; // 전송속도
    char dev_name[128]; // 시리얼포트 노드파일 이름
    char cc; // 데이타 버퍼
    int i = 0;

    if ( argc != 3 )
    {
        printf( " msq_serial [device] [baud]\n" \
        " device : /dev/ttyUSB0 ...\n" \
        " baud : 57600\n" );
        return -1;
    }

    printf( " Serial test start... (%s)\n", __DATE__ ); 

    // 인자를 얻어온다.
    strcpy( dev_name, argv[1] ); // 노드파일 이름
    baud = strtoul( argv[2], NULL, 10 ); // 전송속도

    // 시리얼 포트를 연다
    // 시리얼포트를 1초동안 대기하거나 1바이트 이상의 데이타가 들어오면
    // 깨어나도록 설정한다.
    fd = open_serial( dev_name, baud, 10, 1);
    if ( fd < 0 ) {
        perror("[failed] open serial");
        return -2; 
    }
    printf("open serial socket\n");

    write2serial_port();

    // 시리얼 포트를 닫는다.
    close_serial( fd );
    printf( " Serial test end\n" ); 
    return 0;
}
