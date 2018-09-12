#ifndef SYS_UPDATE_H
#define SYS_UPDATE_H

#include <pthread.h>

#define DISK_PATH	"/mnt/usb1/"

typedef struct
{
	char		cur_version[128];
	char		update_version[128];
	
	pthread_t	disk_tid;
	int			dist_tid_stop;
	int 		disk_interval;		// 检测TF/sd时间间隔
	char		disk_file[512];		// 升级包文件名

	char		remote_addr[256];	// 远程升级服务器地址，建议APP通知下载
	
}sys_update_t;

sys_update_t* sys_update_create();
void sys_update_destory(sys_update_t* update);

int sys_update_disk_info_set(sys_update_t* update, char* file, int interval);
int sys_update_disk_start(sys_update_t* update);
int sys_update_disk_stop(sys_update_t* update);

int sys_update_remote_start(sys_update_t* update);
int sys_update_remote_stop(sys_update_t* update);

#endif