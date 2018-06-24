#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
 
#define KEY_NUM 1

int fd;
unsigned char key_array[KEY_NUM] = {0};
    
void signal_f(int signum)
{
    static unsigned int cnt = 1;
    unsigned int i;

    read(fd, &key_array, sizeof(key_array));

    printf("count:%4d\t",cnt++);
    for(i=0; i<KEY_NUM; i++){
        printf("%02x  ", key_array[i]);
    }
    printf("\n");
}

int main(int argc, char *argv[]) 
{
    int flag;
    
    fd = open("/dev/buttons", O_RDWR);
    if (fd < 0)
    {
        printf("can't open!\n");
    }
    signal(SIGIO, signal_f);
    fcntl(fd, F_SETOWN, getpid());
    flag = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flag|FASYNC);
    while (1)
    {
        sleep(2);
    }
    return 0;
}