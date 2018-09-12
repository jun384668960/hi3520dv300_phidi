#include "sys_update.h"

int main(int argc, char* argv)
{
	sys_update_t* update = sys_update_create();

	sys_update_disk_info_set(update, DISK_PATH"C_300.M01C02S01W01.D5820HAH.update.package", 3);
	sys_update_disk_start(update);
	
	while(1)
	{
		usleep(1000*1000*1000);
	}
	
	sys_update_destory(update);
	
	return 0;
}
