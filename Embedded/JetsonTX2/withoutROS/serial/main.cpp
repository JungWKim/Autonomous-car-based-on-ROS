#include "serial.h"

int fd; // 시리얼포트 파일핸들
int recvBuffer = 101;
int sendBuffer;
int rdcnt; 

void write_port()
{
    while(1)
    {
        // 데이타를 읽어온다.
        if((rdcnt = write( fd, &recvBuffer, sizeof(recvBuffer))) < 0)
        {
            perror("[failed] write");
            exit(1);
        }
        cout << "received data :" << recvBuffer << endl;
        sleep(1);
    }

}

void read_port()
{
    while(1)
    {
        // 데이타를 읽어온다.
        if((rdcnt = read(fd, &sendBuffer, sizeof(sendBuffer))) < 0)
        {
            perror("[failed] read");
            exit(1);
        }
        cout << "send data :" << sendBuffer << endl;
        sleep(1);
    }

}

int main(int argc, char **argv) {

    int baud; // 전송속도
    char dev_name[128]; // 시리얼포트 노드파일 이름
    char cc; // 데이타 버퍼
    int i = 0;
    int thread_id;
    pthread_t p_thread[2];

    if ( argc != 3 )
    {
        printf( " serial [device] [baud]\n" \
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
    std::cout << "open serial socket" << std::endl;

    thread_id = pthread_create(&p_thread[0], NULL, t_function, (void *)p1);
    if (thr_id < 0)
    {
        perror("thread create error : ");
        exit(0);
    }

    // ② 2번 쓰레드 생성
    thr_id = pthread_create(&p_thread[1], NULL, t_function, (void *)p2);
    if (thr_id < 0)
    {
        perror("thread create error : ");
        exit(0);
    }

    // 시리얼 포트를 닫는다.
    close_serial( fd );
    printf( " Serial test end\n" ); 
    return 0;
}
