#ifndef PHIDI_H
#define PHIDI_H

#define MAX_FRAME_LEN 	(1024*1024)
#define HDMI_FREME_MODE	0x10
#define HDMI_RESOLUTION 0x12

typedef enum 
{
	VI_HDMI_READY,
	VI_HDMI_EMPTY,
	VI_HDMI_CHANGE_RES,
	VI_HDMI_CHANGE_EMPTY,
}viHDMI_STATUS;

typedef enum 
{
	VI_RESOLUTION_1920X1080	= 1,
	VI_RESOLUTION_1280X1024	= 5,
	VI_RESOLUTION_1280X800	= 6,
	VI_RESOLUTION_1280X768	= 7,
	VI_RESOLUTION_1280X720	= 2,
	VI_RESOLUTION_720X576	= 3,
	VI_RESOLUTION_720X480	= 4,
	VI_RESOLUTION_640X480	= 8,
	VI_RESOLUTION_UNKOWN	= -1,
}VI_RESOLUTION_E;

typedef enum 
{
	VI_FRAME_MODE_P = 0,
	VI_FRAME_MODE_I = 1
}VI_FRAME_MODE_E;

#endif