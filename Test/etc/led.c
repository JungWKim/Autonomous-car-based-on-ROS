#include <linux/gpio.h>
#include <linux/delay.h>
#define 2

int main()
{
	pinMode(4, OUTPUT);

	while(1)
	{
		digitalWrite(HIGH);
		mdelay(2000);
		digitalWrite(LOW);
		mdelay(2000);
	}

	return 0;
}
