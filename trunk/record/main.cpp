#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/types.h>
#include <string.h>

#include "MP4Encoder.h"
#include "stream_manager.h"
#include "stream_define.h"

#include "utils_log.h"
#include "utils_common.h"

#define 	STORAGE_DIR	"/mnt/usb1/"
int g_W = 1280;
int g_H = 720;

#define KEY_NUM 1
int fd;
unsigned char key_array[KEY_NUM] = {0};
int g_pressed = 0;
int g_recStart = 0;
    
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

MP4EncoderResult AddH264Track(MP4Encoder &encoder, uint8_t *sData, int nSize)
{
	return encoder.MP4AddH264Track(sData, nSize, g_W, g_H);

	return MP4ENCODER_ERROR(MP4ENCODER_E_ADD_VIDEO_TRACK);
}

MP4EncoderResult AddALawTrack(MP4Encoder &encoder)
{
	return encoder.MP4AddALawTrack(NULL, 0);
}

MP4EncoderResult WriteH264Data(MP4Encoder &encoder, uint8_t *sData, frame_info info)
{
	MP4EncoderResult result;

	return encoder.MP4WriteH264Data(sData, info.length, info.pts);
}

MP4EncoderResult WriteALawData(MP4Encoder &encoder, uint8_t *sData, frame_info info)
{
	MP4EncoderResult result;

	return encoder.MP4WriteALawData(sData, info.length, info.pts);
}

int main(int argc, const char *argv[])
{
	init_key_sighandle();
	shm_stream_t* audio_stream = shm_stream_create("rec_audioread", "audiostream", STREAM_MAX_USER, STREAM_MAX_FRAMES, STREAM_AUDIO_MAX_SIZE, SHM_STREAM_READ);
	shm_stream_t* main_stream = shm_stream_create("rec_mainread", "mainstream", STREAM_MAX_USER, STREAM_MAX_FRAMES, STREAM_VIDEO_MAX_SIZE, SHM_STREAM_READ);
	unsigned char* pData = (unsigned char*)malloc(1024*1024);
	while(1)
	{
		while(g_recStart)
		{
			bool Iwait = false;
			bool IwaitOver = false;
			bool vTrackSet = false;
			bool aTrackSet = false;
			MP4EncoderResult result;
			MP4Encoder encoder;
			//根据当前时间取得文件名
			char filename[64] = {0};
			localtime_mp4name_get(STORAGE_DIR, filename);
			LOGI_print("start recording file %s", filename);
			
			result = encoder.MP4CreateFile(filename);
			if(result != MP4ENCODER_ENONE)
			{
				LOGE_print("MP4CreateFile error");
				g_recStart = ~g_recStart;
				continue;
			}
			while(g_recStart)
			{
				frame_info info;
				unsigned char* frame;
				unsigned int length;
				int ret = shm_stream_get(main_stream, &info, &frame, &length);
				if(ret == 0)
				{
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
							result = AddH264Track(encoder, pData, length);
							LOGW_print("AddH264Track error:%d length:%d", result, length);
							vTrackSet = true;
						}
							
						result = WriteH264Data(encoder, pData, info);
						shm_stream_post(main_stream);
						if(result == MP4ENCODER_ERROR(MP4ENCODER_WARN_RECORD_OVER))
						{	
							//直到下一个I帧出现
							IwaitOver = true;
						}
						else if(result != MP4ENCODER_ENONE)
						{
							LOGW_print("WriteH264Data error:%d", result);
							g_recStart = ~g_recStart;
							break;
						}
					}
					
					while(g_recStart)
					{
						ret = shm_stream_get(audio_stream, &info, &frame, &length);
						if(ret == 0)
						{
							if(!vTrackSet)
							{
								shm_stream_post(audio_stream);
							}
							else
							{
								if(!aTrackSet)
								{
									result = AddALawTrack(encoder);
									LOGW_print("AddALawTrack error:%d", result);
								}
								
								memcpy(pData, frame, length);
								result = WriteALawData(encoder, pData, info);
								shm_stream_post(audio_stream);
								if(result != MP4ENCODER_ENONE 
									&& result != MP4ENCODER_ERROR(MP4ENCODER_WARN_RECORD_OVER))
								{
									LOGW_print("WriteALawData error:%d %d", result, shm_stream_remains(audio_stream));
									g_recStart = ~g_recStart;
									break;
								}
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
					usleep(5*1000);
				}
			}
			
			encoder.MP4ReleaseFile();
			LOGI_print("close recording file %s", filename);
		}
		usleep(10*1000);
	}

	free(pData);
	if(main_stream != NULL)
		shm_stream_destory(main_stream);
	if(audio_stream != NULL)
		shm_stream_destory(audio_stream);
	return 0;
}
