#include <iostream>
#include <unistd.h>

using namespace std;

int main()
{
    for(;;) {
        std::system("echo \"---------------------\"");
        std::system("cat /sys/class/thermal/thermal_zone*/temp >> ~/temperature.txt");
        std::system("cat /sys/class/thermal/thermal_zone*/temp");
        sleep(1);
    }
}
