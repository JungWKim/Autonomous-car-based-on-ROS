#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
 
 
#define BUF_SIZE 512
#define MAX_BUF 32
 
using namespace std;
 
int main(int argc, char*argv[])
{
    int server_fd, client_fd;
    char buffer[BUF_SIZE];
    struct sockaddr_in server_addr, client_addr;
    char temp[20];
    int msg_size;
    socklen_t len;
 
 
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {// 
        perror("Server : Can't open stream socket");
        exit(0);
    }
    //server_Addr 을 NULL로 초기화
    memset(&server_addr, 0x00, sizeof(server_addr));
 
    //server_addr 셋팅
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(9999);
 
    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) <0)
    {//bind() 호출
        perror("Server : Can't bind local address");
        exit(0);
    }
 
    if (listen(server_fd, 5) < 0)
    {//소켓을 수동 대기모드로 설정
        perror("Server : Can't listening connect");
        exit(0);
    }

    memset(buffer, 0x00, sizeof(buffer));
    len = sizeof(client_addr);
    cout << "Server : wating connection request.\n";
    client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &len);
    inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, temp, sizeof(temp));
    cout << "Server : %s client connected.\n", temp;

    while (1)
    {
 
        if (client_fd < 0)
        {
            perror("Server : accept failed");
            return 0;
        }
 
        msg_size = read(client_fd, buffer, 1024);
        cout << buffer << endl;
        if (!strcmp(buffer, "fin"))
        {
            cout << "fin\n";
            return 0;
        }
 
 
        memset(buffer, 0x00, sizeof(buffer));
    }
    close(server_fd);
    return 0;
}
