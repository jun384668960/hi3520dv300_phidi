#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>

struct led_desc{
	unsigned int  key_val;
	unsigned char status;
};

int main(int argc, char *argv[]) 
{
    struct led_desc led;
	
    int fd = open("/dev/led", O_RDWR);
    if (fd < 0)
    {
        printf("can't open!\n");
    }

	led.key_val = 0x01;
	led.status = 0;
    while (1)
    {
		write(fd, &led, sizeof(led));
		if(led.status == 0)
			led.status = 1;
		else
			led.status = 0;
		
        sleep(2);
    }

	close(fd);
    return 0;
}