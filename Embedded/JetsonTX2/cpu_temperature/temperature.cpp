#include <iostream>
#include <unistd.h>

using namespace std;

int main()
{
    FILE *fp;
    for(;;) {
        if((fp = fopen("/home/user/temperature.txt", "w")) < 0)
        {
            perror("[failed] fopen file reset");
        }
        fclose(fp);
        std::system("cat /sys/class/thermal/thermal_zone0/temp >> ~/temperature.txt");
        std::system("echo \"\" >> ~/temperature.txt");
        std::system("cat /sys/class/thermal/thermal_zone0/temp");
        std::system("echo \"---------------------\"");
        sleep(1);
    }
}
