#include <iostream>
#include <unistd.h>

using namespace std;

int main()
{
    for(;;) {
        std::system("cat /sys/class/thermal/thermal_zone*/temp >> ./temperature.txt");
        std::system("echo \"---------------------\" >> ./temperature.txt");
        std::system("cat /sys/class/thermal/thermal_zone*/temp");
        std::system("echo \"---------------------\"");
        sleep(1);
    }
}
