/******************************************************************************
  A simple program of Hisilicon HI3531 video encode implementation.
  Copyright (C), 2010-2011, Hisilicon Tech. Co., Ltd.
 ******************************************************************************
    Modification:  2011-2 Created
******************************************************************************/
#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* End of #ifdef __cplusplus */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>

#include "sample_comm.h"
#include "mpi_sys.h"
#include "mpi_vb.h"
#include "mpi_aenc.h"
#include "mpi_ai.h"
#include "mpi_ao.h"
#include "aacenc_lib.h"

#include "utils_log.h"
#include "stream_manager.h"
#include "stream_define.h"
#include "utils_common.h"

VIDEO_NORM_E gs_enNorm = VIDEO_ENCODING_MODE_NTSC;
#define VPSS_BSTR_CHN     		0
#define VPSS_LSTR_CHN     		1
//#define VENC_MAX_CHN_NUM  2

#define MAX_FRAME_LEN (1024*1024)
HANDLE_AACENCODER hAacEncoder;
static PAYLOAD_TYPE_E gs_enPayloadType = PT_G711A;//PT_G711A;//PT_LPCM;
static SAMPLE_VENC_GETSTREAM_PARA_S gs_stVPara;
static SAMPLE_VENC_GETSTREAM_PARA_S gs_stAPara;
static pthread_t gs_VencPid;
static pthread_t gs_AencPid;
static shm_stream_t* g_handle = NULL;
static shm_stream_t* g_audiohandle = NULL;
static unsigned char* g_frame = NULL;

HI_U32 HDMI_H,HDMI_W;

VI_DEV_ATTR_S DEV_ATTR_BT1120_1080P_1MUX_BASE =
{
    /*接口模式*/
    VI_MODE_BT1120_STANDARD,
    /*1、2、4路工作模式*/
    VI_WORK_MODE_1Multiplex,
    /* r_mask    g_mask    b_mask*/
    {0x0000FF00,    0x000000FF},		//dev0 bt1120

	/* 双沿输入时必须设置 */
	VI_CLK_EDGE_SINGLE_UP,		//VI_CLK_EDGE_SINGLE_UP,				//VI_CLK_EDGE_DOUBLE
	
    /*AdChnId*/
    {-1, -1, -1, -1},
    /*enDataSeq, 仅支持YUV格式*/
    VI_INPUT_DATA_UVUV,		//VI_INPUT_DATA_YVYU,
    /*同步信息，对应reg手册的如下配置, --bt1120时序无效*/
    {
    /*port_vsync   port_vsync_neg     port_hsync        port_hsync_neg        */
    VI_VSYNC_FIELD, VI_VSYNC_NEG_HIGH, VI_HSYNC_VALID_SINGNAL,VI_HSYNC_NEG_HIGH,VI_VSYNC_VALID_SINGAL,VI_VSYNC_VALID_NEG_HIGH,

    /*timing信息，对应reg手册的如下配置*/
    /*hsync_hfb    hsync_act    hsync_hhb*/
    {0,            0,        0,
    /*vsync0_vhb vsync0_act vsync0_hhb*/
     0,            0,        0,
    /*vsync1_vhb vsync1_act vsync1_hhb*/
     0,            0,            0}
    },
    /*使用内部ISP*/
    VI_PATH_BYPASS,
    /*输入数据类型*/
    VI_DATA_TYPE_YUV,
     //bDataRev 
    HI_TRUE
};

void FDK_AACLC_Init(HI_VOID)
{    
    if (aacEncOpen(&hAacEncoder, 0, 2) != AACENC_OK) {
		LOGE_print("Unable to open encoder");
	}
    
    //AOT_SBR 64K
    aacEncoder_SetParam(hAacEncoder, AACENC_AOT,AOT_AAC_LC);		//AOT_AAC_LC AOT_PS AOT_SBR
	aacEncoder_SetParam(hAacEncoder, AACENC_BITRATE, 128000);
	aacEncoder_SetParam(hAacEncoder, AACENC_SAMPLERATE, 24000);
	aacEncoder_SetParam(hAacEncoder, AACENC_CHANNELMODE, MODE_2);		//AOUT_CHANS_STEREO
	aacEncoder_SetParam(hAacEncoder, AACENC_GRANULE_LENGTH, 1024);	  //960 performance is better than 1024
		
	//aacEncoder_SetParam(hAacEncoder, AACENC_SBR_MODE, 0);					//Disable Spectral Band Replication
    //aacEncoder_SetParam(hAacEncoder, AACENC_SBR_RATIO, 0);				//close download sampled SBR
    
    if (aacEncEncode(hAacEncoder, NULL, NULL, NULL, NULL) != AACENC_OK) 
	LOGI_print("Unable to initialize the encoder");
}  


HI_S32 Phidi_AOUT_HdmiSet(AIO_ATTR_S stHdmiAoAttr)
{
	HI_S32 s32Ret;
    HI_HDMI_ATTR_S      stAttr;

	s32Ret = HI_MPI_HDMI_SetAVMute(HI_HDMI_ID_0, HI_TRUE);
    if(HI_SUCCESS != s32Ret)
    {
        printf("[Func]:%s [Line]:%d [Info]:%s\n", __FUNCTION__, __LINE__, "failed");
        return HI_FAILURE;
    }
    HI_MPI_HDMI_GetAttr(HI_HDMI_ID_0, &stAttr);

    stAttr.bEnableAudio = HI_TRUE;        /**< if enable audio */
    stAttr.enSoundIntf = HI_HDMI_SND_INTERFACE_I2S; /**< source of HDMI audio, HI_HDMI_SND_INTERFACE_I2S suggested.the parameter must be consistent with the input of AO*/
    stAttr.enSampleRate = stHdmiAoAttr.enSamplerate;        /**< sampling rate of PCM audio,the parameter must be consistent with the input of AO */
    stAttr.u8DownSampleParm = HI_FALSE;    /**< parameter of downsampling  rate of PCM audio,default :0 */
    
    stAttr.enBitDepth = 8 * (stHdmiAoAttr.enBitwidth+1);   /**< bitwidth of audio,default :16,the parameter must be consistent with the config of AO */
    stAttr.u8I2SCtlVbit = 0;        /**< reserved,should be 0, I2S control (0x7A:0x1D) */
    
    stAttr.bEnableAviInfoFrame = HI_TRUE; /**< if enable  AVI InfoFrame*/
    stAttr.bEnableAudInfoFrame = HI_TRUE;; /**< if enable AUDIO InfoFrame*/
    
    
    HI_MPI_HDMI_SetAttr(HI_HDMI_ID_0, &stAttr);

    s32Ret = HI_MPI_HDMI_SetAVMute(HI_HDMI_ID_0, HI_FALSE);
    if(HI_SUCCESS != s32Ret)
    {
        printf("[Func]:%s [Line]:%d [Info]:%s\n", __FUNCTION__, __LINE__, "failed");
        return HI_FAILURE;
    }
    
    printf("HDMI start success.\n");
    return HI_SUCCESS;
}

//	Audio init
HI_S32 Phidi_AENC_Init(HI_VOID)
{
	HI_S32 		s32Ret = 0;
	//HI_S32			i = 0; 
	AUDIO_DEV   AiDev = 0;
	AI_CHN      AiChn	= 0;
	AENC_CHN    AeChn = 0;
	//HI_S32 		fd					= -1;
    //unsigned int i2s_fs_sel = 0;
		
    AIO_ATTR_S stAioAttr;
    stAioAttr.enSamplerate = AUDIO_SAMPLE_RATE_48000;		
	stAioAttr.enBitwidth  = AUDIO_BIT_WIDTH_16;						
	stAioAttr.enWorkmode = AIO_MODE_I2S_SLAVE;
    stAioAttr.enSoundmode = AUDIO_SOUND_MODE_MONO;
    stAioAttr.u32EXFlag = 1;															//扩展成16 位，8bit到16bit 扩展标志只对AI采样精度为8bit 时有效
    stAioAttr.u32FrmNum = 30;
    stAioAttr.u32PtNumPerFrm = 320;
    stAioAttr.u32ChnCnt = 2;										//stereo mode must be 2 
    stAioAttr.u32ClkChnCnt   = 2;
    stAioAttr.u32ClkSel = 0;
		
	//step 1: config audio codec
	/*** INNER AUDIO CODEC ***/
    /*
    s32Ret = SAMPLE_INNER_CODEC_CfgAudio(stAioAttr.enSamplerate); 
    if (HI_SUCCESS != s32Ret)
    {
        LOGE_print("SAMPLE_INNER_CODEC_CfgAudio failed");
        return s32Ret;
    }
	*/
	//step 2: start Ai
    s32Ret = HI_MPI_AI_SetPubAttr(AiDev, &stAioAttr);
    if (s32Ret)
    {
        LOGE_print("HI_MPI_AI_SetPubAttr(%d) failed with %#x", AiDev, s32Ret);
        return HI_FAILURE;
    }
    s32Ret =HI_MPI_AI_Enable(AiDev);
    if (s32Ret)
    {
        LOGE_print("HI_MPI_AI_Enable(%d) failed with %#x", AiDev, s32Ret);
        return HI_FAILURE;
    }
    
    s32Ret =HI_MPI_AI_EnableChn(AiDev,AiChn);
    //s32Ret =HI_MPI_AI_EnableChn(1,0);
    if (s32Ret)
    {
        LOGE_print("HI_MPI_AI_EnableChn(%d,%d) failed with %#x", AiDev, AiChn, s32Ret);
        return -1;    
    }
  	//step 3: start LPCM Aenc
  	
    AENC_CHN_ATTR_S stAencAttr;    
    AENC_ATTR_LPCM_S stAencLpcm;
    
    //memset(&stAencAttr, 0, sizeof(AENC_CHN_ATTR_S));
    //memset(&stAencLpcm, 0, sizeof(AENC_ATTR_LPCM_S));
    stAencAttr.pValue = &stAencLpcm;
    stAencAttr.enType       = gs_enPayloadType;
    stAencAttr.u32BufSize 	= 30;
    stAencAttr.u32PtNumPerFrm	= 320;
	
    //start Audio Encode
    /* create aenc chn*/
    s32Ret = HI_MPI_AENC_CreateChn(AeChn, &stAencAttr);
    if (s32Ret != HI_SUCCESS)
    {
        LOGE_print("%s: HI_MPI_AENC_CreateChn(%d) failed with %#x!", __FUNCTION__,
               AeChn, s32Ret);
        return HI_FAILURE;
    }        
    LOGI_print("after HI_MPI_AENC_CreateChn");
 	 
    /********************************************
      step 4: Aenc bind Ai Chn
    ********************************************/
    MPP_CHN_S stSrcChn,stDestChn;
    
	stSrcChn.enModId = HI_ID_AI;
	stSrcChn.s32DevId = AiDev;
	stSrcChn.s32ChnId = AiChn;
	stDestChn.enModId = HI_ID_AENC;
	stDestChn.s32DevId = 0;
	stDestChn.s32ChnId = AeChn;
	
	s32Ret = HI_MPI_SYS_Bind(&stSrcChn, &stDestChn);
	if (s32Ret != HI_SUCCESS)
	{
		LOGE_print("Ai(%d,%d) bind to AencChn:%d failed!",AiDev , AiChn, AeChn);
		return s32Ret;
}
        
    LOGI_print("Ai(%d,%d) bind to AencChn:%d ok!",AiDev , AiChn, AeChn);
    
    return s32Ret;
}	


/******************************************************************************
* function :  H264 encode
******************************************************************************/
HI_S32 Phidi_VENC_Init(HI_VOID)
{
    //SAMPLE_VI_MODE_E enViMode = SAMPLE_VI_MODE_8_720P;
    // HI_S32 s32VpssGrpCnt = 8;
    VB_CONF_S stVbConf;
    VPSS_GRP VpssGrp;
    VPSS_CHN VpssChn = 0;
    VENC_CHN VencChn = 0;
    VPSS_CHN_MODE_S stVpssChnMode;
    HI_S32 s32Ret;
    HI_U32 u32BlkSize;
	VI_DEV_ATTR_S  stViDevAttr;
	
	int chn=0;
	int dev=0;
    /******************************************
     step  1: init variable 
    ******************************************/
    memset(&stVbConf,0,sizeof(VB_CONF_S));

    u32BlkSize = SAMPLE_COMM_SYS_CalcPicVbBlkSize(gs_enNorm,\
                PIC_HD1080, SAMPLE_PIXEL_FORMAT, SAMPLE_SYS_ALIGN_WIDTH,COMPRESS_MODE_SEG);
    stVbConf.u32MaxPoolCnt = 64;
    
    stVbConf.astCommPool[0].u32BlkSize = u32BlkSize;
    //stVbConf.astCommPool[0].u32BlkCnt = u32ViChnCnt * 6;
    stVbConf.astCommPool[0].u32BlkCnt = 18;
    memset(stVbConf.astCommPool[0].acMmzName,0,
    sizeof(stVbConf.astCommPool[0].acMmzName));

    /******************************************
     step 2: mpp system init. 
    ******************************************/
    s32Ret = SAMPLE_COMM_SYS_Init(&stVbConf);
    if (HI_SUCCESS != s32Ret)
    {
        LOGE_print("system init failed with %d!", s32Ret);
    }

    /******************************************
     step 3: start vi dev & chn to capture
    ******************************************/
     
    HDMI_W = 1920;
    HDMI_H = 1080;   	
    int HDMImode=0;
    
    int g_fd = open("/dev/it6801", 0);
    if(g_fd < 0)
    {
        LOGE_print("Open it6801 error!");
    }
	else
	{	
		HDMImode = ioctl(g_fd, 0x10, 0);
		if (HDMImode < 0)
	 	{
    		LOGE_print(" VI_SCAN_INTERLACED read error!");
		}
		if(HDMImode > 0){ 
			HDMImode = 5;		//stViDevAttr.enScanMode = VI_SCAN_INTERLACED;
			LOGI_print("interlace mode");    
		
		} else {
			HDMImode = ioctl(g_fd, 0x12, 0);
		}	
    }
    close(g_fd);

	LOGI_print("resolution mode : %d",HDMImode); 
	if(HDMImode == 2) {   	//720P
		HDMI_W = 1280;
	    HDMI_H = 720;
	    LOGI_print("resolution : 720P"); 
    }	
    if(HDMImode == 3) {			//576P
		HDMI_W = 720;
        HDMI_H = 576;
        LOGI_print("resolution : 576P");			
    }
    if(HDMImode == 4) {			//480P
 		HDMI_W = 720;
		HDMI_H = 480;   			
		LOGI_print("resolution : 480P");
    }	
    if(HDMImode == 1) {
    	LOGI_print("resolution : 1080P");
    }	
	
    memset(&stViDevAttr,0,sizeof(stViDevAttr));
    memcpy(&stViDevAttr,&DEV_ATTR_BT1120_1080P_1MUX_BASE,sizeof(stViDevAttr));     
   	LOGI_print("before HI_MPI_VI_SetDevAttr");
   
    s32Ret = HI_MPI_VI_SetDevAttr(dev, &stViDevAttr);
    if (s32Ret != HI_SUCCESS)
    {
        LOGE_print("HI_MPI_VI_SetDevAttr failed with %#x!", s32Ret);
    }

    s32Ret = HI_MPI_VI_EnableDev(dev);
    if (s32Ret != HI_SUCCESS)
    {
        LOGE_print("HI_MPI_VI_EnableDev failed with %#x!", s32Ret);
    }

	/*** Start VI Chn ***/
	VI_CHN_BIND_ATTR_S stChnBindAttr;
	s32Ret = HI_MPI_VI_GetChnBind(chn, &stChnBindAttr);
	if (s32Ret != HI_SUCCESS)
    {
        LOGE_print("HI_MPI_VI_GetChnBind(%d) failed with %#x!", chn, s32Ret);
    }

	stChnBindAttr.ViDev = dev;	
    stChnBindAttr.ViWay = 0;
    s32Ret = HI_MPI_VI_BindChn(chn, &stChnBindAttr);
    if (s32Ret != HI_SUCCESS)
    {
        LOGE_print("HI_MPI_VI_BindChn(%d) failed with %#x!", chn, s32Ret);
    }
   
    ///////////////////////////////////////////////////////
 	//SAMPLE_COMM_VI_StartChn
    ///////////////////////////////////////////////////////
 	VI_CHN_ATTR_S stChnAttr;
    
	stChnAttr.stCapRect.s32X = 0;
	stChnAttr.stCapRect.s32Y = 0;
	stChnAttr.enCapSel = VI_CAPSEL_BOTH;                      
	stChnAttr.enPixFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420;   /* sp420 or sp422 */
	stChnAttr.bMirror = HI_FALSE;                             
	stChnAttr.bFlip   = HI_FALSE;  
	stChnAttr.stCapRect.u32Width= HDMI_W;
	stChnAttr.stCapRect.u32Height= HDMI_H;	
	stChnAttr.stDestSize.u32Width= HDMI_W;
	stChnAttr.stDestSize.u32Height= HDMI_H;
	stChnAttr.enScanMode = VI_SCAN_PROGRESSIVE;		//VI_SCAN_INTERLACED		
	stChnAttr.s32SrcFrameRate = -1;
	stChnAttr.s32DstFrameRate = -1;

	if(5==HDMImode) {
		stChnAttr.enScanMode = VI_SCAN_INTERLACED;
	}	
		
    s32Ret = HI_MPI_VI_SetChnAttr(chn, &stChnAttr);
    if (HI_SUCCESS != s32Ret)
    {
		LOGE_print("call HI_MPI_VI_SetChnAttr failed with %#x", s32Ret);
	} else {
		LOGI_print("HI_MPI_VI_SetChnAttr(%d) ready", chn); 
    }	
    
    s32Ret = HI_MPI_VI_EnableChn(chn);
	if (s32Ret != HI_SUCCESS)
	{
		LOGE_print("Enable chn failed with error code %#x!", s32Ret);
	}
    
    /******************************************
     step 4: start vpss and vi bind vpss
    ******************************************/
    
    VPSS_GRP_ATTR_S stGrpAttr;
    VPSS_GRP_PARAM_S stVpssParam;
    
    stGrpAttr.u32MaxW = HDMI_W;
    stGrpAttr.u32MaxH = HDMI_H;
    stGrpAttr.enPixFmt = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
        
    stGrpAttr.bIeEn = HI_FALSE;
    stGrpAttr.bNrEn = HI_FALSE;
    stGrpAttr.bDciEn = HI_FALSE;
    stGrpAttr.bHistEn = HI_FALSE;
    stGrpAttr.bEsEn = HI_FALSE;
    stGrpAttr.enDieMode = VPSS_DIE_MODE_NODIE;
    
    VpssGrp = 0;
    /*** create vpss group ***/
    s32Ret = HI_MPI_VPSS_CreateGrp(VpssGrp, &stGrpAttr);
    if (s32Ret != HI_SUCCESS)
    {
        LOGE_print("HI_MPI_VPSS_CreateGrp failed with %#x!", s32Ret);
        //return HI_FAILURE;
    }

    /*** set vpss param ***/
    s32Ret = HI_MPI_VPSS_GetGrpParam(VpssGrp, &stVpssParam);
    if (s32Ret != HI_SUCCESS)
    {
        LOGE_print("failed with %#x!", s32Ret);
        //return HI_FAILURE;
    }
        
    stVpssParam.u32IeStrength = 0;
    s32Ret = HI_MPI_VPSS_SetGrpParam(VpssGrp, &stVpssParam);
    if (s32Ret != HI_SUCCESS)
    {
        LOGE_print("failed with %#x!", s32Ret);
        //return HI_FAILURE;
    }
    
    VPSS_CHN_ATTR_S stVpssChnAttr;
    /* Set Vpss Chn attr */
    stVpssChnAttr.bSpEn = HI_FALSE;
    stVpssChnAttr.bUVInvert = HI_FALSE;
    stVpssChnAttr.bBorderEn = HI_TRUE;
    stVpssChnAttr.stBorder.u32Color = 0xff00;
    stVpssChnAttr.stBorder.u32LeftWidth = 2;
    stVpssChnAttr.stBorder.u32RightWidth = 2;
    stVpssChnAttr.stBorder.u32TopWidth = 2;
    stVpssChnAttr.stBorder.u32BottomWidth = 2;
            
    s32Ret = HI_MPI_VPSS_SetChnAttr(VpssGrp, VpssChn, &stVpssChnAttr);
    if (s32Ret != HI_SUCCESS)
    {
    	LOGE_print("HI_MPI_VPSS_SetChnAttr failed with %#x", s32Ret);
        //return HI_FAILURE;
    }
    
  	s32Ret = HI_MPI_VPSS_GetChnMode(VpssGrp,VpssChn,&stVpssChnMode);
  	if(s32Ret != HI_SUCCESS)
	{
		LOGE_print("HI_MPI_VPSS_GetChnMode failed with %#x", s32Ret);
	}
  
    stVpssChnMode.enChnMode = VPSS_CHN_MODE_USER;
    stVpssChnMode.enPixelFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    stVpssChnMode.u32Width = HDMI_W;
    stVpssChnMode.u32Height = HDMI_H; 
    //memset(&stVpssChnMode.stAspectRatio,0, sizeof(ASPECT_RATIO_S)); 
   
    s32Ret = HI_MPI_VPSS_SetChnMode(VpssGrp, VpssChn, &stVpssChnMode);
    if (s32Ret != HI_SUCCESS)
    {
    	LOGE_print("HI_MPI_VPSS_SetChnMode failed with %#x", s32Ret);
        //return HI_FAILURE;
    }
    
    s32Ret = HI_MPI_VPSS_EnableChn(VpssGrp, VpssChn);
    if (s32Ret != HI_SUCCESS)
    {
    	LOGE_print("HI_MPI_VPSS_EnableChn failed with %#x", s32Ret);
        //return HI_FAILURE;
    }
    
    /*** start vpss group ***/
    s32Ret = HI_MPI_VPSS_StartGrp(VpssGrp);
    if (s32Ret != HI_SUCCESS)
    {
        LOGE_print("HI_MPI_VPSS_StartGrp failed with %#x", s32Ret);
    	//return HI_FAILURE;
    }
    
    MPP_CHN_S stSrcChn;
    MPP_CHN_S stDestChn;
    
    //SAMPLE_COMM_VI_BindVpss
    stSrcChn.enModId = HI_ID_VIU;
    stSrcChn.s32DevId = 0;
    stSrcChn.s32ChnId = chn;		//ViChn;
    
    stDestChn.enModId = HI_ID_VPSS;
    stDestChn.s32DevId = dev;
    stDestChn.s32ChnId = VpssChn;
  
    s32Ret = HI_MPI_SYS_Bind(&stSrcChn, &stDestChn);
    if (s32Ret != HI_SUCCESS)
    {
    	LOGE_print("HI_MPI_SYS_Bind(%d) failed with %#x!", VpssChn, s32Ret);
        //return HI_FAILURE;
    } else {
    	LOGI_print("Vi_BindVpss Success");
    }	
    	
    /******************************************
     step 5: start stream venc
    ******************************************/
    //VpssGrp = 0;
    //VpssChn = 1;
    //VencChn = 1;
   
	//SAMPLE_COMM_VENC_Start
    VENC_CHN_ATTR_S stVencChnAttr;
    VENC_ATTR_H264_S stH264Attr;
    VENC_ATTR_H264_CBR_S    stH264Cbr;
    //SIZE_S stPicSize;
    
    stVencChnAttr.stVeAttr.enType = PT_H264;

	stH264Attr.u32MaxPicWidth = HDMI_W;
    stH264Attr.u32MaxPicHeight = HDMI_H;
    stH264Attr.u32PicWidth = HDMI_W;//the picture width
    stH264Attr.u32PicHeight = HDMI_H;//the picture height
    stH264Attr.u32BufSize  = 2073600;//stream buffer size
    stH264Attr.u32Profile  = 0;//0: baseline; 1:MP; 2:HP;  3:svc_t 
    stH264Attr.bByFrame = HI_TRUE;//get stream mode is slice mode or frame mode?
	stH264Attr.u32BFrameNum = 0;// 0: not support B frame; >=1: number of B frames 
	stH264Attr.u32RefNum = 0;// 0: default; number of refrence frame
	memcpy(&stVencChnAttr.stVeAttr.stAttrH264e, &stH264Attr, sizeof(VENC_ATTR_H264_S));
			
	stVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    stH264Cbr.u32Gop            = (VIDEO_ENCODING_MODE_PAL== gs_enNorm)?25:30;
    stH264Cbr.u32StatTime       = 1; // stream rate statics time(s) 
    stH264Cbr.u32SrcFrmRate     = (VIDEO_ENCODING_MODE_PAL== gs_enNorm)?25:30;// input (vi) frame rate 
    stH264Cbr.fr32DstFrmRate 		= (VIDEO_ENCODING_MODE_PAL== gs_enNorm)?25:30;// target frame rate
	stH264Cbr.u32BitRate = 1024*5;//1024*5;			
	stH264Cbr.u32FluctuateLevel = 0; // average bit rate 
	memcpy(&stVencChnAttr.stRcAttr.stAttrH264Cbr, &stH264Cbr, sizeof(VENC_ATTR_H264_CBR_S));
		
    s32Ret = HI_MPI_VENC_CreateChn(VencChn, &stVencChnAttr);
    //s32Ret = HI_MPI_VENC_CreateChn(chn, &stVencChnAttr);
    if (HI_SUCCESS != s32Ret)
    {
        LOGE_print("HI_MPI_VENC_CreateChn [%d] faild with %#x!",\
                VencChn, s32Ret);
    }
    else
    {
    	LOGI_print("HI_MPI_VENC_CreateChn [%d] success",VencChn);
    }

    s32Ret = HI_MPI_VENC_StartRecvPic(VencChn);
    //s32Ret = HI_MPI_VENC_StartRecvPic(chn);
    if (HI_SUCCESS != s32Ret)
    {
        LOGE_print("HI_MPI_VENC_StartRecvPic(%d) faild with%#x!", VencChn, s32Ret);
    }
    
	//SAMPLE_COMM_VENC_BindVpss
    
    stSrcChn.enModId = HI_ID_VPSS;
    stSrcChn.s32DevId = VpssGrp;
    stSrcChn.s32ChnId = VpssChn;

    stDestChn.enModId = HI_ID_VENC;
    stDestChn.s32DevId = 0;
    stDestChn.s32ChnId = VencChn;

    s32Ret = HI_MPI_SYS_Bind(&stSrcChn, &stDestChn);		
    if (s32Ret != HI_SUCCESS)
    {
        LOGE_print("failed with %#x!", s32Ret);
        //return HI_FAILURE;
    } else {
    	LOGI_print("VENC_BindVpss vpssChn=%d vencChn=%d", VpssChn, VencChn);
    }

	return s32Ret;
    /******************************************
     step 6: stream venc process -- get stream, then save it to file. 
    ******************************************/
    //s32Ret = SAMPLE_COMM_VENC_StartGetStream(s32VpssGrpCnt*2);
    //if (HI_SUCCESS != s32Ret)
    //{
    //    printf("Start Venc failed!");
        //goto END_VENC_8_720p_3;
    //}
	
    LOGE_print("please press twice ENTER to exit this sample");
    getchar();
    getchar();

    /******************************************
     step 7: exit process
    ******************************************/
    SAMPLE_COMM_VENC_StopGetStream();
   
    
	SAMPLE_COMM_VENC_UnBindVpss(VencChn, VpssGrp, VpssChn);
	SAMPLE_COMM_VENC_SnapStop(VencChn);
	//SAMPLE_COMM_VI_UnBindVpss(enViMode);

    SAMPLE_COMM_SYS_Exit();
    
    return s32Ret;
}


HI_S32 Phidi_AOUT_HDMI_Init(HI_VOID)
{
    HI_S32 s32Ret;
    AUDIO_DEV   AiDev = SAMPLE_AUDIO_AI_DEV;
    AI_CHN      AiChn = 1;
    AUDIO_DEV   AoDev = SAMPLE_AUDIO_HDMI_AO_DEV;
    AO_CHN      AoChn = 0;

    AIO_ATTR_S stHdmiAoAttr;
	AUDIO_RESAMPLE_ATTR_S stAoReSampleAttr;

	stHdmiAoAttr.enSamplerate   = AUDIO_SAMPLE_RATE_48000;
    stHdmiAoAttr.enBitwidth     = AUDIO_BIT_WIDTH_16;
    stHdmiAoAttr.enWorkmode     = AIO_MODE_I2S_MASTER;
    stHdmiAoAttr.enSoundmode    = AUDIO_SOUND_MODE_MONO;
    stHdmiAoAttr.u32EXFlag      = 1;
    stHdmiAoAttr.u32FrmNum      = 30;
    stHdmiAoAttr.u32PtNumPerFrm = SAMPLE_AUDIO_PTNUMPERFRM;
    stHdmiAoAttr.u32ChnCnt      = 2;
	stHdmiAoAttr.u32ClkChnCnt   = 2;
    stHdmiAoAttr.u32ClkSel      = 0;  
	
	/* ao 8k -> 48k */
	stAoReSampleAttr.u32InPointNum	= SAMPLE_AUDIO_PTNUMPERFRM;
	stAoReSampleAttr.enInSampleRate = AUDIO_SAMPLE_RATE_32000;
	stAoReSampleAttr.enOutSampleRate = AUDIO_SAMPLE_RATE_48000;

	s32Ret =HI_MPI_AI_EnableChn(AiDev, AiChn);
	if (s32Ret)
	{
		LOGE_print("HI_MPI_AI_EnableChn(%d,%d) failed with %#x", AiDev, AiChn, s32Ret);
		return -1;	  
	}
		
	Phidi_AOUT_HdmiSet(stHdmiAoAttr);

    s32Ret = HI_MPI_AO_SetPubAttr(AoDev, &stHdmiAoAttr);
    if (HI_SUCCESS != s32Ret)
    {
        LOGE_print("HI_MPI_AO_SetPubAttr(%d) failed with %#x!", AoDev, s32Ret);
        return HI_FAILURE;
    }

    s32Ret = HI_MPI_AO_Enable(AoDev);
    if (HI_SUCCESS != s32Ret)
    {
        LOGE_print("HI_MPI_AO_Enable(%d) failed with %#x!", AoDev, s32Ret);
        return HI_FAILURE;
    }

	s32Ret = HI_MPI_AO_EnableChn(AoDev, AoChn);
    if (HI_SUCCESS != s32Ret)
    {
        LOGE_print("HI_MPI_AO_EnableChn(%d) failed with %#x!", AoChn, s32Ret);
        return HI_FAILURE;
    }

	/* AI to AO channel */
	s32Ret = SAMPLE_COMM_AUDIO_CreatTrdAiAo(AiDev, AiChn, AoDev, AoChn);
	if (s32Ret != HI_SUCCESS)
	{
		LOGE_print("SAMPLE_COMM_AUDIO_CreatTrdAiAo s32Ret:%d", s32Ret);
		return HI_FAILURE;
	}

	return HI_TRUE;
}

HI_S32 Phidi_VOUT_HDMI_Init(HI_VOID)
{
	VPSS_GRP VpssGrp;
	VPSS_CHN VpssChn_VoHD0 = VPSS_CHN0;

	VO_DEV VoDev;
	VO_LAYER VoLayer;
	VO_CHN VoChn;
	VO_PUB_ATTR_S stVoPubAttr;
	VO_VIDEO_LAYER_ATTR_S stLayerAttr;
	SAMPLE_VO_MODE_E enVoMode, enPreVoMode;

	HI_S32 i;
	HI_S32 s32Ret = HI_SUCCESS;
	HI_U32 u32WndNum;
	
	printf("start vo HD0.\n");
	VoDev = SAMPLE_VO_DEV_DHD0;
	VoLayer = SAMPLE_VO_LAYER_VHD0;
	u32WndNum = 1;
	enVoMode = VO_MODE_1MUX;

	stVoPubAttr.enIntfSync = VO_OUTPUT_1080P60;
	stVoPubAttr.enIntfType = VO_INTF_HDMI|VO_INTF_VGA;
	stVoPubAttr.u32BgColor = 0x00000000;
	s32Ret = SAMPLE_COMM_VO_StartDev(VoDev, &stVoPubAttr);
	if (HI_SUCCESS != s32Ret)
	{
		LOGE_print("Start SAMPLE_COMM_VO_StartDev failed!");
		return HI_FAILURE;
	}

	memset(&(stLayerAttr), 0 , sizeof(VO_VIDEO_LAYER_ATTR_S));
	s32Ret = SAMPLE_COMM_VO_GetWH(stVoPubAttr.enIntfSync, \
		&stLayerAttr.stImageSize.u32Width, \
		&stLayerAttr.stImageSize.u32Height, \
		&stLayerAttr.u32DispFrmRt);
	if (HI_SUCCESS != s32Ret)
	{
		LOGE_print("Start SAMPLE_COMM_VO_GetWH failed!");
		return HI_FAILURE;
	}

	stLayerAttr.enPixFormat = SAMPLE_PIXEL_FORMAT;
	stLayerAttr.stDispRect.s32X 	  = 0;
	stLayerAttr.stDispRect.s32Y 	  = 0;
	stLayerAttr.stDispRect.u32Width   = stLayerAttr.stImageSize.u32Width;
	stLayerAttr.stDispRect.u32Height  = stLayerAttr.stImageSize.u32Height;
	s32Ret = SAMPLE_COMM_VO_StartLayer(VoLayer, &stLayerAttr);
	if (HI_SUCCESS != s32Ret)
	{
		LOGE_print("Start SAMPLE_COMM_VO_StartLayer failed!");
		return HI_FAILURE;
	}

	s32Ret = SAMPLE_COMM_VO_StartChn(VoLayer, enVoMode);
	if (HI_SUCCESS != s32Ret)
	{
		LOGE_print("Start SAMPLE_COMM_VO_StartChn failed!");
		return HI_FAILURE;
	}

	/* if it's displayed on HDMI, we should start HDMI */
	if (stVoPubAttr.enIntfType & VO_INTF_HDMI)
	{
		if (HI_SUCCESS != SAMPLE_COMM_VO_HdmiStart(stVoPubAttr.enIntfSync))
		{
			LOGE_print("Start SAMPLE_COMM_VO_HdmiStart failed!");
			return HI_FAILURE;
		}
		
		LOGI_print("HDMI start success.\n");
	}

	for(i=0;i<u32WndNum;i++)
	{
		VoChn = i;
		VpssGrp = i;
		
		s32Ret = SAMPLE_COMM_VO_BindVpss(VoDev,VoChn,VpssGrp,VpssChn_VoHD0);
		if (HI_SUCCESS != s32Ret)
		{
			LOGE_print("Start VO failed!");
			return HI_FAILURE;
		}
	}

	enPreVoMode = enVoMode;
	VoDev = SAMPLE_VO_DEV_DHD0;
	VoLayer = SAMPLE_VO_LAYER_VHD0;
	enVoMode = VO_MODE_1MUX;
	s32Ret= HI_MPI_VO_SetAttrBegin(VoLayer);
	if (HI_SUCCESS != s32Ret)
	{
		LOGE_print("Start VO failed!");
		return HI_FAILURE;
	}
	
	s32Ret = SAMPLE_COMM_VO_StopChn(VoLayer, enPreVoMode);
	if (HI_SUCCESS != s32Ret)
	{
		LOGE_print("Start VO failed!");
		return HI_FAILURE;
	}

	s32Ret = SAMPLE_COMM_VO_StartChn(VoLayer, enVoMode);
	if (HI_SUCCESS != s32Ret)
	{
		LOGE_print("Start VO failed!");
		return HI_FAILURE;
	}
	s32Ret= HI_MPI_VO_SetAttrEnd(VoLayer);
	if (HI_SUCCESS != s32Ret)
	{
		LOGE_print("Start VO failed!");
		return HI_FAILURE;
	}

	return HI_TRUE;
}

void COMM_VENC_UseStream(VENC_CHN VeChn, PAYLOAD_TYPE_E enType, VENC_STREAM_S *pstStream)
{
	//do some thing
	if(VeChn == 0)
	{
		HI_S32 total_length = 0;
		HI_S32 i;

		for (i = 0; i < pstStream->u32PackCount; i++)
	    {
	    	HI_U8* data = pstStream->pstPack[i].pu8Addr+pstStream->pstPack[i].u32Offset;
			HI_S32 len = pstStream->pstPack[i].u32Len-pstStream->pstPack[i].u32Offset;
			if(total_length + len > MAX_FRAME_LEN)
				len = MAX_FRAME_LEN - total_length;

			memcpy(&g_frame[total_length], data, len);
			total_length += len;
	    }
		if(g_handle != NULL)
		{
			frame_info info;
			info.type = enType;
			if(enType == PT_H264)
			{
				info.key = pstStream->pstPack[pstStream->u32PackCount-1].DataType.enH264EType;
			}
			info.pts = pstStream->pstPack[pstStream->u32PackCount-1].u64PTS;
			info.length = total_length;
			
			int ret = shm_stream_put(g_handle, info, g_frame, total_length);
			if(ret != 0)
			{
//				LOGE_print("shm_stream_put error");
			}
			else
			{
//				LOGI_print("shm_stream_put video info.lenght:%d info.pts:%llu", info.length, info.pts);
			}
		}
	}
}

void COMM_AENC_UseStream(HI_S32 AeChn, AUDIO_STREAM_S *pstStream)
{
	if(AeChn == 0 && gs_enPayloadType == PT_G711A)
	{
		if(g_audiohandle != NULL)
		{
			frame_info info;
			info.type = gs_enPayloadType;
			info.key = 1;
			info.pts = pstStream->u64TimeStamp;
			info.length = pstStream->u32Len - 4;
			
			int ret = shm_stream_put(g_audiohandle, info, pstStream->pStream + 4, info.length);
			if(ret != 0)
			{
//				LOGE_print("shm_stream_put error");
			}
			else
			{
//				LOGI_print("shm_stream_put audio info.lenght:%d info.pts:%llu", info.length, info.pts);
			}
		}
	}
}

/******************************************************************************
* funciton : get stream from each channels and save them
******************************************************************************/
HI_VOID* COMM_VENC_GetVencStreamProc()
{
    HI_S32 i;
    HI_S32 maxfd = 0;
    struct timeval TimeoutVal;
    fd_set read_fds;
    HI_S32 s32Ret;
	//PAYLOAD_TYPE_E enPayLoadType[2];
	VENC_STREAM_S stStream;
	VENC_CHN_STAT_S stStat;
	VENC_STREAM_BUF_INFO_S stStreamBufInfo;
	HI_S32 VencFd[VENC_MAX_CHN_NUM];
   
    i=0;
    VENC_CHN VencChn = i;
		
    VencFd[i] = HI_MPI_VENC_GetFd(i);
    if (VencFd[i] < 0)
    {
        LOGE_print("HI_MPI_VENC_GetFd failed with %#x!", VencFd[i]);
    }
    if (maxfd <= VencFd[i])
    {
        maxfd = VencFd[i];
    }
		
    while (HI_TRUE)
    {
        FD_ZERO(&read_fds);
        FD_SET(VencFd[i], &read_fds);
	
        TimeoutVal.tv_sec  = 5;
        TimeoutVal.tv_usec = 0;
        s32Ret = select(maxfd + 1, &read_fds, NULL, NULL, &TimeoutVal);
        if (s32Ret < 0)
        {
            LOGE_print("select failed!");
            break;
        }
        else if (s32Ret == 0)
        {
            LOGE_print("get venc stream time out, continue");
//            break;
			continue;
        }
        else
        {
            if (FD_ISSET(VencFd[i], &read_fds))
            {
				i=0;

				s32Ret = HI_MPI_VENC_GetStreamBufInfo (VencChn,&stStreamBufInfo);
				if (HI_SUCCESS != s32Ret)
				{
					//return HI_FAILURE;
				}
								
				memset(&stStream, 0, sizeof(stStream));
				s32Ret = HI_MPI_VENC_Query(i, &stStat);//query stream state
				if (HI_SUCCESS != s32Ret)
				{
					LOGE_print("HI_MPI_VENC_Query chn[%d] failed with %#x!", i, s32Ret);
					break;
				}

				//step 2.2 : malloc corresponding number of pack nodes.
				stStream.pstPack = (VENC_PACK_S*)malloc(sizeof(VENC_PACK_S) * stStat.u32CurPacks);
				if (NULL == stStream.pstPack)
				{
					LOGE_print("malloc stream pack failed!");
					break;
				}

				// step 2.3 : call mpi to get one-frame stream
				//printf("%d  ",stStat.u32CurPacks);
				stStream.u32PackCount = stStat.u32CurPacks;
				s32Ret = HI_MPI_VENC_GetStream(i, &stStream, HI_TRUE);
				if (HI_SUCCESS != s32Ret)
				{
					free(stStream.pstPack);
					stStream.pstPack = NULL;
					LOGE_print("HI_MPI_VENC_GetStream failed with %#x!", \
					    s32Ret);
					break;
				}

				// step 2.4 : save frame to file
				COMM_VENC_UseStream(i, PT_H264, &stStream);

				if (HI_SUCCESS != s32Ret)
				{
					free(stStream.pstPack);
					stStream.pstPack = NULL;
					LOGE_print("save stream failed!");
					break;
				}			
				// step 2.5 : release stream
				s32Ret = HI_MPI_VENC_ReleaseStream(i, &stStream);
				if (HI_SUCCESS != s32Ret)
				{
					free(stStream.pstPack);
					stStream.pstPack = NULL;
					break;
				}
				// step 2.6 : free pack nodes
				free(stStream.pstPack);
				stStream.pstPack = NULL;			
            }
        }
    }

	return NULL;
}

/******************************************************************************
* function : get stream from Aenc, send it  to Adec & save it to file
******************************************************************************/
HI_VOID* COMM_AENC_GetAencStreamProc()
{
    HI_S32 maxfd = 0;
    struct timeval TimeoutVal;
    fd_set read_fds;
    HI_S32 s32Ret;
	AUDIO_STREAM_S stStream;
	HI_S32 AencFd;
   	AENC_CHN AeChn = 0;
   	
//   	AACENC_BufDesc in_buf = { 0 }, out_buf = { 0 };
//	AACENC_InArgs in_args = { 0 };
//	AACENC_OutArgs out_args = { 0 };
//	int in_identifier = IN_AUDIO_DATA;
//	int in_size, in_elem_size;
//	int out_identifier = OUT_BITSTREAM_DATA;
//	int out_size, out_elem_size;
//	void *in_ptr, *out_ptr;
//	//uint8_t outbuf[5120];
//	HI_U8 outbuf[5120];
   		
    AencFd = HI_MPI_AENC_GetFd(AeChn);
    if (AencFd < 0)
    {
        LOGE_print("HI_MPI_AENC_GetFd failed with %#x!", AencFd);
    }
    if (maxfd <= AencFd)
    {
        maxfd = AencFd;
    }
        
    LOGI_print("maxfd = %d",maxfd);
    while (HI_TRUE)
    {
        FD_ZERO(&read_fds);
        FD_SET(AencFd, &read_fds);
				
        TimeoutVal.tv_sec  = 5;
        TimeoutVal.tv_usec = 0;
        s32Ret = select(maxfd + 1, &read_fds, NULL, NULL, &TimeoutVal);
        if (s32Ret < 0)
        {
            LOGW_print("select failed!");
            break;
        }
        else if (s32Ret == 0)
        {
            LOGW_print("get aenc stream time out, continue");
//            break;
			continue;
        }
        else
        {
        	if (FD_ISSET(AencFd, &read_fds))
            {
				s32Ret = HI_MPI_AENC_GetStream(AeChn, &stStream, HI_FALSE);
        		if (HI_SUCCESS != s32Ret )
        		{
            		LOGE_print("%s: HI_MPI_AENC_GetStream(%d), failed with %#x!",\
                   		__FUNCTION__, AeChn, s32Ret);
        		}

//				in_ptr = stStream.pStream;
//				in_size = stStream.u32Len;
//				in_elem_size = 2;

//				in_args.numInSamples = in_size/2;
//				in_buf.numBufs = 1;
//				in_buf.bufs = &in_ptr;
//				in_buf.bufferIdentifiers = &in_identifier;
//				in_buf.bufSizes = &in_size;
//				in_buf.bufElSizes = &in_elem_size;

//				out_ptr = outbuf;
//				out_size = 1024;//sizeof(outbuf);
//				out_elem_size = 1;
//				out_buf.numBufs = 1;
//				out_buf.bufs = &out_ptr;
//				out_buf.bufferIdentifiers = &out_identifier;
//				out_buf.bufSizes = &out_size;
//				out_buf.bufElSizes = &out_elem_size;

//				if (aacEncEncode(hAacEncoder, &in_buf, &out_buf, &in_args, &out_args) != AACENC_OK) {
//					LOGE_print("Encoding failed");
//					return NULL;
//				}
				// save audio stream to file 
//				static FILE* pAAC = NULL;
//				if(pAAC==NULL) pAAC=fopen("./test.aac","wb");
//				if(pAAC!=NULL) fwrite(stStream.pStream+4,1,stStream.u32Len-4, pAAC);
//				int num = 0; 
//				for(num; num<out_buf.numBufs)
//				{
//					void* buff = out_buf.bufs[num];
//					int len = out_buf.bufSizes[num];
//					if(pAAC!=NULL) fwrite(buff,1,len, pAAC);
//				}
				
        		//fwrite(stStream.pStream,1,stStream.u32Len, pstAencCtl->pfd);
        		//fwrite(stStream.pStream,1,stStream.u32Len, fp);
        		//fwrite(outbuf,1,1024, fp);
				COMM_AENC_UseStream(AeChn, &stStream);
        	
        		// finally you must release the stream
        		s32Ret = HI_MPI_AENC_ReleaseStream(AeChn, &stStream);
        		if (HI_SUCCESS != s32Ret )
        		{
            		LOGE_print("%s: HI_MPI_AENC_ReleaseStream(%d), failed with %#x!",\
                   		__FUNCTION__, AeChn, s32Ret);

        		}										
            }
        }
    }

	return NULL;
}

/******************************************************************************
* function    : main()
* Description : video venc sample
******************************************************************************/
int main(int argc, char *argv[])
{
    HI_S32 s32Ret;
	g_frame = (unsigned char*)malloc(MAX_FRAME_LEN);
	g_handle = shm_stream_create("write", "mainstream", STREAM_MAX_USER, STREAM_MAX_FRAMES, STREAM_VIDEO_MAX_SIZE, SHM_STREAM_WRITE);
	g_audiohandle = shm_stream_create("write", "audiostream", STREAM_MAX_USER, STREAM_MAX_FRAMES, STREAM_AUDIO_MAX_SIZE, SHM_STREAM_WRITE);

    s32Ret = Phidi_VENC_Init();
    s32Ret = Phidi_AENC_Init();
//	FDK_AACLC_Init();
	Phidi_VOUT_HDMI_Init();
	Phidi_AOUT_HDMI_Init();
//	Phidi_HDMI_Start();

    gs_stVPara.bThreadStart = HI_TRUE;
    gs_stVPara.s32Cnt = 1;
    pthread_create(&gs_VencPid, 0, COMM_VENC_GetVencStreamProc, (HI_VOID*)&gs_stVPara);  
    
	s32Ret=HI_MPI_VI_EnableChn(0);
	gs_stAPara.bThreadStart = HI_TRUE;
  	gs_stAPara.s32Cnt = 1;
	pthread_create(&gs_AencPid, 0, COMM_AENC_GetAencStreamProc, (HI_VOID*)&gs_stAPara);  
	
    while(1)
	{
		usleep(1000*1000*10);
	}

    SAMPLE_COMM_VENC_StopGetStream();
    LOGI_print("SAMPLE_COMM_VENC_StopGetStream");

    SAMPLE_COMM_VENC_Stop(0);
    //SAMPLE_COMM_VI_Stop
    HI_MPI_VI_DisableChn(0);
    HI_MPI_VI_DisableDev(0);
    
    SAMPLE_COMM_SYS_Exit();

	shm_stream_destory(g_handle);
	shm_stream_destory(g_audiohandle);
	free(g_frame);
    if (HI_SUCCESS == s32Ret)
        LOGW_print("program exit normally!");
    else
        LOGW_print("program exit abnormally!");
    exit(s32Ret);
	
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
