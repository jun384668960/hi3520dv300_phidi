#include <stdio.h>
#include <time.h>
#include <sys/vfs.h>
#include <sys/sysinfo.h>
#include <string.h>

#include "utils_common.h"

void localtime_string_get(char* tstr)
{
	char *wday[]={"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
	time_t timep;
	struct tm *p;

	tzset();
	time(&timep);
	p=localtime(&timep);
	
	sprintf(tstr,"%d/%02d/%02d %s %02d:%02d:%02d", (1900+p->tm_year),(1+p->tm_mon), p->tm_mday
		, wday[p->tm_wday],p->tm_hour, p->tm_min, p->tm_sec);
}

void localtime_mp4name_get(char* dir, char* filename)
{
	time_t timep;
	struct tm *p;
	tzset();
	time(&timep);
	p=localtime(&timep);
	sprintf(filename,"%s/%04d-%02d-%02d %02d-%02d-%02d.mp4", dir, (1900+p->tm_year),(1+p->tm_mon), p->tm_mday
		, p->tm_hour, p->tm_min, p->tm_sec);
}

int exec_cmd(const char *cmd)   
{   
	if(cmd == NULL)
		return -1;

    FILE *fp = NULL;
    if((fp = popen(cmd, "r")) == NULL)
    {
        printf("Fail to popen\n");
        return -1;
    }
    pclose(fp);
	
	return 0;
} 

//execute shell command
int exec_cmd_ex(const char *cmd, char* res, int max)
{   
	if(cmd == NULL || res == NULL || max <= 0)
		return -1;
	
    FILE *pp = popen(cmd, "r");
    if(!pp) 
	{
        printf("error, cannot popen cmd: %s\n", cmd);
        return -1;
    }

	int length;
    char tmp[1024] = {0};
	
	length = max;
	if(max > 1024) length = 1024;
		
    while(fgets(tmp, length, pp) != NULL) 
	{
        if(tmp[length-1] == '\n') {
            tmp[length-1] = '\0';
        }

        strcpy(res, tmp);
    }

    pclose(pp);

    return strlen(res);
}


unsigned long long get_system_tf_freeKb(char* dir)
{
	int ret;
	if(dir == NULL) return 0;
	
	struct statfs diskInfo;
	ret = statfs(dir, &diskInfo);
	if(ret == 0)
	{
		unsigned long long totalBlocks = diskInfo.f_bsize;
		unsigned long long freeDisk = diskInfo.f_bfree*totalBlocks;
		return freeDisk/1024LL;
	}
	
	return 0;
}

unsigned long get_system_mem_freeKb()
{
	int error;  
  	struct sysinfo s_info;  
	
	error = sysinfo(&s_info);  
	if(error == 0)
	{
		return s_info.freeram/1024L;
	}
	return 0;
}

int64_t get_tick_count()
{
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	return (int64_t)now.tv_sec * 1000000 + now.tv_nsec / 1000;
}

int pthread_create_4m(pthread_t* pt_id, void *(*proc)(void *), void* arg)
{
    pthread_attr_t attr;
    
    pthread_attr_init (&attr); 
	int stacksize = (4 << 10 ) << 10;
	pthread_attr_setstacksize(&attr, stacksize);
	
    int ret = pthread_create(pt_id,&attr,proc,arg);
    if (ret != 0)
    {
      pthread_attr_destroy (&attr); 
	  printf("pthread_create error %s\n", strerror(ret));
      return -1;
    }
    pthread_attr_destroy (&attr); 
    
    return 0;
}

