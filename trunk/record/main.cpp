#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/types.h>
#include <string.h>
#include <pthread.h>

#include "stream_manager.h"
#include "stream_define.h"

#include "utils_log.h"
#include "utils_common.h"
#include "mp4_muxer.h"

#include "nalu_utils.hh"
#include "h264_stream.h"

int g_rec_time = 60*60*1000;

#define KEY_NUM 1
int fd;
unsigned char key_array[KEY_NUM] = {0};
int g_pressed = 0;			
int g_mounted = 0;			//是否mount成功U盘
int g_recStart = 0;
int g_recStopping = 0;
int g_ledPidStop = 0;

#define STORAGE_DIR "/mnt/usb1/"

struct led_desc{
	unsigned int  key_val;
	unsigned char status;
};

int is_usb_mount()
{
	char result[1024] = {0};
	return exec_cmd_ex("cat /proc/mounts | grep /mnt/usb1", result, 1024);
}
void signal_f(int signum)
{
    unsigned int i;

    read(fd, &key_array, sizeof(key_array));

    for(i=0; i<KEY_NUM; i++){
		if(key_array[i] == 0x01)
		{
			g_pressed = 1;
		}
		else
		{
			if(g_pressed == 1)
			{
				g_recStart = ~g_recStart;
			}
			g_pressed = 0;
		}
        LOGI_print("%02x  g_recStart:%d", key_array[i], g_recStart);
    }
}

void init_key_sighandle()
{
    int flag;
    
    fd = open("/dev/buttons", O_RDWR);
    if (fd < 0)
    {
        LOGE_print("can't open!");
    }
    signal(SIGIO, signal_f);
    fcntl(fd, F_SETOWN, getpid());
    flag = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flag|FASYNC);
}

void* init_led_sighandle(void*)
{
	struct led_desc led;
	int	   	stop_sec = 0;
	
    int fd = open("/dev/led", O_RDWR);
    if (fd < 0)
    {
        printf("can't open!\n");
    }

	led.key_val = 0x01;
	led.status = 1;
	while(!g_ledPidStop)
	{	
		if(is_usb_mount())
		{
			g_mounted = 1;
			if(g_recStart != 0)
			{
				if(led.status == 0)
					led.status = 1;
				else
					led.status = 0;
				write(fd, &led, sizeof(led));
			}
			else
			{
				if(g_recStopping == 1)
				{
					if(led.status == 0)
						led.status = 1;
					else
						led.status = 0;
					write(fd, &led, sizeof(led));

					usleep(450*1000);
					
					stop_sec+=1;
					if(stop_sec > 10)
					{
						g_recStopping = 0;
						stop_sec = 0;
					}
					continue;
				}
				else
				{
					led.status = 0;
					write(fd, &led, sizeof(led));
				}
			}
		}
		else
		{
			g_recStopping = 0;
			stop_sec = 0;
			g_mounted = 0;
			led.status = 1;
			write(fd, &led, sizeof(led));
		}
		
		sleep(1);
	}

	close(fd);
}

int main(int argc, const char *argv[])
{
	pthread_t led_pid;
	
	init_key_sighandle();
	pthread_create(&led_pid, 0, init_led_sighandle, NULL);
	
	shm_stream_t* audio_stream = shm_stream_create("rec_audioread", "audiostream", STREAM_MAX_USER, STREAM_MAX_FRAMES, STREAM_AUDIO_MAX_SIZE, SHM_STREAM_READ);
	shm_stream_t* main_stream = shm_stream_create("rec_mainread", "mainstream", STREAM_MAX_USER, STREAM_MAX_FRAMES, STREAM_VIDEO_MAX_SIZE, SHM_STREAM_READ);
	unsigned char* pData = (unsigned char*)malloc(1024*1024);
	while(1)
	{
		while(g_recStart && g_mounted == 1)
		{
			bool Iwait = false;
			bool IwaitOver = false;
			bool vTrackSet = false;
			bool aTrackSet = false;
			int  result;
			int  rec_ref = 0;
			
			//根据当前时间取得文件名
			char filename[64] = {0};
			localtime_mp4name_get(STORAGE_DIR, filename);
			LOGI_print("start recording file %s", filename);
			
			MP4Mux_Open(filename);
			
			while(g_recStart)
			{
				frame_info info;
				unsigned char* frame;
				unsigned int length;
				int ret = shm_stream_get(main_stream, &info, &frame, &length);
				if(ret == 0)
				{
//					LOGI_print("shm_stream_get viedeo info.lenght:%d info.pts:%llu", info.length, info.pts);
					//如果还没有取得I帧，并且当前非I帧
					if(!Iwait && info.key == 1)
					{
						LOGW_print("Wait for I frame Iwait:%d info.key:%d", Iwait, info.key);
						shm_stream_post(main_stream);
					}
					else
					{
						if(IwaitOver && info.key == 5)
						{
							LOGW_print("WriteH264Data IwaitOver:%d", IwaitOver);
							break;
						}
							
						memcpy(pData, frame, length);
						Iwait = true;
						if(!vTrackSet)
						{
							AM_VIDEO_INFO pvInfo;
							AM_AUDIO_INFO paInfo;
							
							paInfo.chunkSize = 1024;
							paInfo.format = MF_AAC;
							paInfo.pktPtsIncr = 1024 * 90000 / 48000;
							paInfo.sampleRate = 48000;
							paInfo.sampleSize = 16;         
							paInfo.channels = 1;
							
							MP4Mux_GetVideoInfo(pData, length, 60, &pvInfo);

							NALU_t nalu;
							get_annexb_nalu(pData, length, &nalu);
							
							h264_stream_t* h4 = h264_new();
							h264_configure_parse(h4, nalu.buf, nalu.len, H264_SPS);
							printf("width:%d, height:%d framerate:%f\n", h4->info->width, h4->info->height, h4->info->max_framerate);

							pvInfo.rate = 60;
							pvInfo.width = h4->info->width;
							pvInfo.height = h4->info->height;
							h264_free(h4);
							
							MP4Mux_OnInfo(&pvInfo, &paInfo);

							vTrackSet = true;
							aTrackSet = true;

							rec_ref = info.pts/1000;
						}
							
						int rec_time = info.pts/1000 - rec_ref;
//						LOGI_print("record time:%d max:%d", rec_time, g_rec_time);
						if(info.key == 5 && rec_time > g_rec_time)
						{
							LOGW_print("record time:%d > max:%d", rec_time, g_rec_time);
							break;
						}
						
						result = MP4Mux_WriteVideoData(pData, length, info.pts/1000);
						shm_stream_post(main_stream);
						if(result != 0)
						{	
							//直到下一个I帧出现
							IwaitOver = true;
						}
					}
					
					while(g_recStart)
					{
						ret = shm_stream_get(audio_stream, &info, &frame, &length);
						if(ret == 0)
						{
//							LOGI_print("shm_stream_get audio info.lenght:%d info.pts:%llu", info.length, info.pts);
							if(!vTrackSet && !aTrackSet)
							{
								shm_stream_post(audio_stream);
							}
							else
							{
								memcpy(pData, frame, length);
								MP4Mux_WriteAudioData(pData, length, info.pts/1000);
								shm_stream_post(audio_stream);
							}
						}
						else
						{
							break;
						}
					}
				}
				else
				{
					usleep(1000);
				}
			}
			
			MP4Mux_Close();
			g_recStopping = 1;
			sync();
			LOGI_print("close recording file %s", filename);
		}
		usleep(10*1000);
	}

	free(pData);
	if(main_stream != NULL)
		shm_stream_destory(main_stream);
	if(audio_stream != NULL)
		shm_stream_destory(audio_stream);

	g_ledPidStop = 1;
	pthread_join(led_pid, NULL);
	return 0;
}
