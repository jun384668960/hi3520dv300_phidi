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



#include "aacenc_lib.h"

HANDLE_AACENCODER hAacEncoder;

VIDEO_NORM_E gs_enNorm = VIDEO_ENCODING_MODE_NTSC;
#define SAMPLE_YUV_D1_FILEPATH         "SAMPLE_420_D1.yuv"
#define VPSS_BSTR_CHN     		0
#define VPSS_LSTR_CHN     		1
//#define VENC_MAX_CHN_NUM  2

static int g_fd = -1;
static pthread_t gs_VencPid;
static SAMPLE_VENC_GETSTREAM_PARA_S gs_stPara;
static pthread_t gs_AencPid;

HI_U32 HDMI_H,HDMI_W;

FILE *pFile;


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


/******************************************************************************
* function : to process abnormal case                                         
******************************************************************************/
void SAMPLE_VENC_HandleSig(HI_S32 signo)
{
    if (SIGINT == signo || SIGTSTP == signo)
    {
        SAMPLE_COMM_SYS_Exit();
        printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
    }
    exit(-1);
}

/******************************************************************************
* function : to process abnormal case - the case of stream venc
******************************************************************************/
void SAMPLE_VENC_StreamHandleSig(HI_S32 signo)
{

    if (SIGINT == signo || SIGTSTP == signo)
    {
        SAMPLE_COMM_SYS_Exit();
        printf("\033[0;31mprogram exit abnormally!\033[0;39m\n");
    }

    exit(0);
}

void FDK_AACLC_Init(HI_VOID)
{    
     if (aacEncOpen(&hAacEncoder, 0, 2) != AACENC_OK) {
				printf("Unable to open encoder\n");
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
				printf("Unable to initialize the encoder\r\n");
}  

//	Audio init
HI_S32 Phidi_AENC_Init(HI_VOID)
{
		HI_S32 		s32Ret = 0;
		//HI_S32			i = 0; 
		AUDIO_DEV   AiDev = 1;
		AI_CHN      AiChn	= 0;
		AENC_CHN    AeChn = 0;
		//HI_S32 		fd					= -1;
    //unsigned int i2s_fs_sel = 0;
		
    AIO_ATTR_S stAioAttr;
    stAioAttr.enSamplerate = AUDIO_SAMPLE_RATE_48000;		
		stAioAttr.enBitwidth  = AUDIO_BIT_WIDTH_16;						
		stAioAttr.enWorkmode = AIO_MODE_I2S_SLAVE;
    stAioAttr.enSoundmode = AUDIO_SOUND_MODE_STEREO;
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
        printf("%s:SAMPLE_INNER_CODEC_CfgAudio failed\n", __FUNCTION__);
        return s32Ret;
    }
		*/
		
		//step 2: start Ai
    s32Ret = HI_MPI_AI_SetPubAttr(AiDev, &stAioAttr);
    if (s32Ret)
    {
        printf("%s: HI_MPI_AI_SetPubAttr(%d) failed with %#x\n", __FUNCTION__, AiDev, s32Ret);
        return HI_FAILURE;
    }
    s32Ret =HI_MPI_AI_Enable(AiDev);
    if (s32Ret)
    {
        printf("%s: HI_MPI_AI_Enable(%d) failed with %#x\n", __FUNCTION__, AiDev, s32Ret);
        return HI_FAILURE;
    }
    
    s32Ret =HI_MPI_AI_EnableChn(AiDev,AiChn);
    //s32Ret =HI_MPI_AI_EnableChn(1,0);
    if (s32Ret)
    {
        printf("%s: HI_MPI_AI_EnableChn(%d,%d) failed with %#x\n",__FUNCTION__, AiDev, AiChn, s32Ret);
        return -1;    
    }
    
  
  	//step 3: start LPCM Aenc
  	
    AENC_CHN_ATTR_S stAencAttr;    
    AENC_ATTR_LPCM_S stAencLpcm;
    
    //memset(&stAencAttr, 0, sizeof(AENC_CHN_ATTR_S));
    //memset(&stAencLpcm, 0, sizeof(AENC_ATTR_LPCM_S));
  
    
    stAencAttr.pValue = &stAencLpcm;
    stAencAttr.enType       = PT_LPCM;
    stAencAttr.u32BufSize 	= 30;
    stAencAttr.u32PtNumPerFrm	= 1024;
  
   

    //start Audio Encode
    /* create aenc chn*/
    s32Ret = HI_MPI_AENC_CreateChn(AeChn, &stAencAttr);
    if (s32Ret != HI_SUCCESS)
    {
            printf("%s: HI_MPI_AENC_CreateChn(%d) failed with %#x!\n", __FUNCTION__,
                   AeChn, s32Ret);
            return HI_FAILURE;
    }        
    printf("after HI_MPI_AENC_CreateChn\n");
 	 
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
				printf("Ai(%d,%d) bind to AencChn:%d failed!!!\n",AiDev , AiChn, AeChn);
				return s32Ret;
		}
        
    printf("Ai(%d,%d) bind to AencChn:%d ok!\n",AiDev , AiChn, AeChn);
    
    return s32Ret;
}	

AUDIO_STREAM_S *pStream;

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
        printf("system init failed with %d!\n", s32Ret);
    }

    /******************************************
     step 3: start vi dev & chn to capture
    ******************************************/
     
    HDMI_W = 1920;
    HDMI_H = 1080;   	
    int HDMImode=0;
    
    g_fd = open("/dev/it6801", 0);
    if(g_fd < 0)
    {
        	printf("Open it6801 error!\n");
    }
		else
		{	
			  //printf("Open it6801 success!\n");
				HDMImode = ioctl(g_fd, 0x10, 0);
    		if (HDMImode < 0)
   	 		{
        		printf(" VI_SCAN_INTERLACED read error!\n");
    		}
    		if(HDMImode > 0){ 
    				HDMImode = 5;		//stViDevAttr.enScanMode = VI_SCAN_INTERLACED;
    				printf("interlace mode\n");    
    		
    		} else {
    				HDMImode = ioctl(g_fd, 0x12, 0);
    		}	
    }
    close(g_fd);


		printf("resolution mode : %d\n",HDMImode); 
		if(HDMImode == 2) {   	//720P
    				HDMI_W = 1280;
            HDMI_H = 720;
            printf("resolution : 720P\n"); 
    }	
    if(HDMImode == 3) {			//576P
    				HDMI_W = 720;
            HDMI_H = 576;
            printf("resolution : 576P\n");			
    }
    if(HDMImode == 4) {			//480P
    	 			HDMI_W = 720;
       			HDMI_H = 480;   			
       			printf("resolution : 480P\n");
       			
    }	
    if(HDMImode == 1) {
    	printf("resolution : 1080P\n");
    }	
	
		

    memset(&stViDevAttr,0,sizeof(stViDevAttr));
    memcpy(&stViDevAttr,&DEV_ATTR_BT1120_1080P_1MUX_BASE,sizeof(stViDevAttr));     
   	printf("before HI_MPI_VI_SetDevAttr\n");
   
    s32Ret = HI_MPI_VI_SetDevAttr(dev, &stViDevAttr);
    if (s32Ret != HI_SUCCESS)
    {
        printf("HI_MPI_VI_SetDevAttr failed with %#x!\n", s32Ret);
       
    }

    s32Ret = HI_MPI_VI_EnableDev(dev);
    if (s32Ret != HI_SUCCESS)
    {
        printf("HI_MPI_VI_EnableDev failed with %#x!\n", s32Ret);
       
    }

		/*** Start VI Chn ***/
		VI_CHN_BIND_ATTR_S stChnBindAttr;
		s32Ret = HI_MPI_VI_GetChnBind(chn, &stChnBindAttr);
		if (s32Ret != HI_SUCCESS)
    {
        printf("HI_MPI_VI_GetChnBind(%d) failed with %#x!\n", chn, s32Ret);
       
    }
		stChnBindAttr.ViDev = dev;	
    stChnBindAttr.ViWay = 0;
    s32Ret = HI_MPI_VI_BindChn(chn, &stChnBindAttr);
     if (s32Ret != HI_SUCCESS)
    {
        printf("HI_MPI_VI_BindChn(%d) failed with %#x!\n", chn, s32Ret);
     
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
            printf("call HI_MPI_VI_SetChnAttr failed with %#x\n", s32Ret);
            
    } else {
    				printf("HI_MPI_VI_SetChnAttr(%d) ready\n", chn);
            
    }	
    
    s32Ret = HI_MPI_VI_EnableChn(chn);
		if (s32Ret != HI_SUCCESS)
		{
				printf("Enable chn failed with error code %#x!\n", s32Ret);
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
            SAMPLE_PRT("HI_MPI_VPSS_CreateGrp failed with %#x!\n", s32Ret);
            //return HI_FAILURE;
    }

    /*** set vpss param ***/
    s32Ret = HI_MPI_VPSS_GetGrpParam(VpssGrp, &stVpssParam);
    if (s32Ret != HI_SUCCESS)
    {
            SAMPLE_PRT("failed with %#x!\n", s32Ret);
            //return HI_FAILURE;
    }
        
    stVpssParam.u32IeStrength = 0;
    s32Ret = HI_MPI_VPSS_SetGrpParam(VpssGrp, &stVpssParam);
    if (s32Ret != HI_SUCCESS)
    {
            SAMPLE_PRT("failed with %#x!\n", s32Ret);
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
    		SAMPLE_PRT("HI_MPI_VPSS_SetChnAttr failed with %#x\n", s32Ret);
        //return HI_FAILURE;
    }
    
  
  	s32Ret = HI_MPI_VPSS_GetChnMode(VpssGrp,VpssChn,&stVpssChnMode);
  	if(s32Ret != HI_SUCCESS)
		{
				SAMPLE_PRT("HI_MPI_VPSS_GetChnMode failed with %#x\n", s32Ret);
		}
  
    stVpssChnMode.enChnMode = VPSS_CHN_MODE_USER;
    stVpssChnMode.enPixelFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    stVpssChnMode.u32Width = HDMI_W;
    stVpssChnMode.u32Height = HDMI_H; 
    //memset(&stVpssChnMode.stAspectRatio,0, sizeof(ASPECT_RATIO_S)); 
   
    s32Ret = HI_MPI_VPSS_SetChnMode(VpssGrp, VpssChn, &stVpssChnMode);
    if (s32Ret != HI_SUCCESS)
    {
    		SAMPLE_PRT("HI_MPI_VPSS_SetChnMode failed with %#x\n", s32Ret);
        //return HI_FAILURE;
    }
    
    s32Ret = HI_MPI_VPSS_EnableChn(VpssGrp, VpssChn);
    if (s32Ret != HI_SUCCESS)
    {
    		SAMPLE_PRT("HI_MPI_VPSS_EnableChn failed with %#x\n", s32Ret);
        //return HI_FAILURE;
    }
    
     
    /*** start vpss group ***/
    s32Ret = HI_MPI_VPSS_StartGrp(VpssGrp);
    if (s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("HI_MPI_VPSS_StartGrp failed with %#x\n", s32Ret);
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
    		SAMPLE_PRT("HI_MPI_SYS_Bind(%d) failed with %#x!\n", VpssChn, s32Ret);
        //return HI_FAILURE;
    } else {
    		SAMPLE_PRT("Vi_BindVpss\n");
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
        SAMPLE_PRT("HI_MPI_VENC_CreateChn [%d] faild with %#x!\n",\
                VencChn, s32Ret);
    }
    else
    	{SAMPLE_PRT("HI_MPI_VENC_CreateChn [%d] success\n",VencChn);
    		}

    s32Ret = HI_MPI_VENC_StartRecvPic(VencChn);
    //s32Ret = HI_MPI_VENC_StartRecvPic(chn);
    if (HI_SUCCESS != s32Ret)
    {
        SAMPLE_PRT("HI_MPI_VENC_StartRecvPic(%d) faild with%#x!\n", VencChn, s32Ret);
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
        printf("failed with %#x!\n", s32Ret);
        //return HI_FAILURE;
    } else {
    	SAMPLE_PRT("VENC_BindVpss vpssChn=%d vencChn=%d \n", VpssChn, VencChn);
    	
    }


		
		return s32Ret;

    /******************************************
     step 6: stream venc process -- get stream, then save it to file. 
    ******************************************/
    //s32Ret = SAMPLE_COMM_VENC_StartGetStream(s32VpssGrpCnt*2);
    //if (HI_SUCCESS != s32Ret)
    //{
    //    printf("Start Venc failed!\n");
        //goto END_VENC_8_720p_3;
    //}
	
    

		

    printf("please press twice ENTER to exit this sample\n");
    getchar();
    getchar();

    /******************************************
     step 7: exit process
    ******************************************/
    SAMPLE_COMM_VENC_StopGetStream();
   
    
		SAMPLE_COMM_VENC_UnBindVpss(VencChn, VpssGrp, VpssChn);
		SAMPLE_COMM_VENC_SnapStop(VencChn);
	//SAMPLE_COMM_VI_UnBindVpss(enViMode);
END_VENC_8_720p_2:	
    //SAMPLE_COMM_VPSS_Stop(s32VpssGrpCnt, VPSS_MAX_CHN_NUM);
END_VENC_8_720p_1:	
   // SAMPLE_COMM_VI_Stop(enViMode);
END_VENC_8_720p_0:	
    SAMPLE_COMM_SYS_Exit();
    
    return s32Ret;
}

void push_video_h264(unsigned char * data, size_t size){
	fwrite(data, 1, size, pFile);
	//printf("h264:%d\n", size);
}

/******************************************************************************
* funciton : save H264 stream
******************************************************************************/
HI_S32 HI_COMM_VENC_SaveH264(HI_S32 s32Chn, VENC_STREAM_S *pstStream)
{
	HI_U8 *pu8Addr;
	HI_U32 u32Len;
	
	HI_S32 i;

	for(i = 0; i < pstStream->u32PackCount; i++)
	{
		pu8Addr=pstStream->pstPack[i].pu8Addr+pstStream->pstPack[i].u32Offset;
		u32Len = pstStream->pstPack[i].u32Len-pstStream->pstPack[i].u32Offset;
		
		push_video_h264(pu8Addr, u32Len);
		
	}
	
	return HI_SUCCESS;
}

/******************************************************************************
* funciton : get stream from each channels and save them
******************************************************************************/
void COMM_VENC_GetVencStreamProc()
{
    HI_S32 i;
    HI_S32 maxfd = 0;
    struct timeval TimeoutVal;
    fd_set read_fds;
    HI_S32 s32Ret;
		//PAYLOAD_TYPE_E enPayLoadType[2];
		VENC_STREAM_S stStream;
		VENC_CHN_STAT_S stStat;
    
		HI_S32 s32VideoChnTotal = 1;
		HI_S32 s32AudioChnTotal = 2;
		
		VENC_STREAM_BUF_INFO_S stStreamBufInfo;
		HI_U32 u32Left;
		HI_U32 u32SrcPhyAddr, u32DestPhyAddr;

		HI_S32 VencFd[VENC_MAX_CHN_NUM];
   
    i=0;
    VENC_CHN VencChn = i;
		
    VencFd[i] = HI_MPI_VENC_GetFd(i);
    if (VencFd[i] < 0)
    {
            printf("HI_MPI_VENC_GetFd failed with %#x!\n", VencFd[i]);
            //return HI_FAILURE;
    }
    if (maxfd <= VencFd[i])
    {
            maxfd = VencFd[i];
    }
        
       
		HI_S32 counter =0;
		
    while (HI_TRUE)
    {
        FD_ZERO(&read_fds);
        FD_SET(VencFd[i], &read_fds);
	
        TimeoutVal.tv_sec  = 5;
        TimeoutVal.tv_usec = 0;
        s32Ret = select(maxfd + 1, &read_fds, NULL, NULL, &TimeoutVal);
        if (s32Ret < 0)
        {
            printf("select failed!\n");
            break;
        }
        else if (s32Ret == 0)
        {
            printf("get venc stream time out, exit thread\n");
            break;
        }
        else
        {
           //for (i = 0; i < s32VideoChnTotal; i++)
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
                         SAMPLE_PRT("HI_MPI_VENC_Query chn[%d] failed with %#x!\n", i, s32Ret);
                         break;
                     }
                     
                     //step 2.2 : malloc corresponding number of pack nodes.
                     stStream.pstPack = (VENC_PACK_S*)malloc(sizeof(VENC_PACK_S) * stStat.u32CurPacks);
                     if (NULL == stStream.pstPack)
                     {
                         SAMPLE_PRT("malloc stream pack failed!\n");
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
                         SAMPLE_PRT("HI_MPI_VENC_GetStream failed with %#x!\n", \
                                s32Ret);
                         break;
                     }
					
                     // step 2.4 : save frame to file   
                     s32Ret =HI_COMM_VENC_SaveH264(i, &stStream);
                     
                     if (HI_SUCCESS != s32Ret)
                     {
                         free(stStream.pstPack);
                         stStream.pstPack = NULL;
                         SAMPLE_PRT("save stream failed!\n");
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
    }

    //return HI_SUCCESS;
}

/******************************************************************************
* function : get stream from Aenc, send it  to Adec & save it to file
******************************************************************************/
void COMM_AENC_GetAencStreamProc()
{
    HI_S32 i;
    HI_S32 maxfd = 0;
    struct timeval TimeoutVal;
    fd_set read_fds;
    HI_S32 s32Ret;
	
		AUDIO_STREAM_S stStream;
		
		HI_S32 AencFd;
   	AENC_CHN AeChn = 0;
   	
   	AACENC_BufDesc in_buf = { 0 }, out_buf = { 0 };
		AACENC_InArgs in_args = { 0 };
		AACENC_OutArgs out_args = { 0 };
		int in_identifier = IN_AUDIO_DATA;
		int in_size, in_elem_size;
		int out_identifier = OUT_BITSTREAM_DATA;
		int out_size, out_elem_size;
		void *in_ptr, *out_ptr;
		//uint8_t outbuf[5120];
		HI_U8 outbuf[5120];
   		
    AencFd = HI_MPI_AENC_GetFd(AeChn);
    if (AencFd < 0)
    {
            printf("HI_MPI_AENC_GetFd failed with %#x!\n", AencFd);
    }
    if (maxfd <= AencFd)
    {
            maxfd = AencFd;
    }
        
    printf("maxfd = %d\n",maxfd);

    while (HI_TRUE)
    {
        FD_ZERO(&read_fds);
        FD_SET(AencFd, &read_fds);
				

        TimeoutVal.tv_sec  = 5;
        TimeoutVal.tv_usec = 0;
        s32Ret = select(maxfd + 1, &read_fds, NULL, NULL, &TimeoutVal);
        if (s32Ret < 0)
        {
            printf("select failed!\n");
            break;
        }
        else if (s32Ret == 0)
        {
            printf("get aenc stream time out, exit thread\n");
            break;
        }
        else
        {
        		if (FD_ISSET(AencFd, &read_fds))
            {
								s32Ret = HI_MPI_AENC_GetStream(AeChn, &stStream, HI_FALSE);
            		if (HI_SUCCESS != s32Ret )
            		{
                		printf("%s: HI_MPI_AENC_GetStream(%d), failed with %#x!\n",\
                       __FUNCTION__, AeChn, s32Ret);
                
            		}

								in_ptr = stStream.pStream;
								in_size = stStream.u32Len;
								in_elem_size = 2;
		

								in_args.numInSamples = in_size/2;
								in_buf.numBufs = 1;
								in_buf.bufs = &in_ptr;
								in_buf.bufferIdentifiers = &in_identifier;
								in_buf.bufSizes = &in_size;
								in_buf.bufElSizes = &in_elem_size;

								out_ptr = outbuf;
								out_size = 1024;//sizeof(outbuf);
								out_elem_size = 1;
								out_buf.numBufs = 1;
								out_buf.bufs = &out_ptr;
								out_buf.bufferIdentifiers = &out_identifier;
								out_buf.bufSizes = &out_size;
								out_buf.bufElSizes = &out_elem_size;

								if (aacEncEncode(hAacEncoder, &in_buf, &out_buf, &in_args, &out_args) != AACENC_OK) {
										SAMPLE_PRT("Encoding failed\n");
										return -1;
								}
            
            		// save audio stream to file 
            		//fwrite(stStream.pStream,1,stStream.u32Len, pstAencCtl->pfd);
            		//fwrite(stStream.pStream,1,stStream.u32Len, fp);
            		//fwrite(outbuf,1,1024, fp);
			
								
            		// finally you must release the stream
            		s32Ret = HI_MPI_AENC_ReleaseStream(AeChn, &stStream);
            		if (HI_SUCCESS != s32Ret )
            		{
                		printf("%s: HI_MPI_AENC_ReleaseStream(%d), failed with %#x!\n",\
                       	__FUNCTION__, AeChn, s32Ret);

            		}										
                
            }
						
        }
    }

    //return HI_SUCCESS;
}

/******************************************************************************
* function    : main()
* Description : video venc sample
******************************************************************************/
int main(int argc, char *argv[])
{
    HI_S32 s32Ret;

 		//signal(SIGPIPE, SIG_IGN);  //not key issue
    //signal(SIGINT, SAMPLE_VENC_HandleSig);
    //signal(SIGTERM, SAMPLE_VENC_HandleSig);
   
    s32Ret = Phidi_VENC_Init();
    s32Ret = Phidi_AENC_Init();
    FDK_AACLC_Init();
    
  

		pFile= fopen("test.h264", "wb"); 
		//pFile= fopen("test.aac", "wb"); 

    gs_stPara.bThreadStart = HI_TRUE;
    gs_stPara.s32Cnt = 1;
    pthread_create(&gs_VencPid, 0, COMM_VENC_GetVencStreamProc, (HI_VOID*)&gs_stPara);  
    
		s32Ret=HI_MPI_VI_EnableChn(0);
		
		
		//gs_stPara.bThreadStart = HI_TRUE;
  	//gs_stPara.s32Cnt = 1;
		//pthread_create(&gs_AencPid, 0, COMM_AENC_GetAencStreamProc, (HI_VOID*)&gs_stPara);  
	
    
    printf("please press twice ENTER to exit this sample\n");
    
    
    getchar();
    getchar();
    
    fclose(pFile);
    
  
    
    SAMPLE_COMM_VENC_StopGetStream();
    printf("SAMPLE_COMM_VENC_StopGetStream\n");
    /*
    if (HI_TRUE == gs_stPara.bThreadStart)
    {
    		printf("SAMPLE_COMM_VENC_StopGetStream\n");
        gs_stPara.bThreadStart = HI_FALSE;
        pthread_join(gs_VencPid, 0);
    }
    */
    
    //SAMPLE_COMM_VENC_UnBindVpss(0, 0, 0);
		//SAMPLE_COMM_VENC_SnapStop(0);
    
    //SAMPLE_COMM_VENC_UnBindVpss(0,0,0);
    //SAMPLE_COMM_VENC_Stop(0);
    
    SAMPLE_COMM_VENC_Stop(0);
    //SAMPLE_COMM_VI_Stop
    HI_MPI_VI_DisableChn(0);
    HI_MPI_VI_DisableDev(0);
    
    SAMPLE_COMM_SYS_Exit();
    
    if (HI_SUCCESS == s32Ret)
        printf("program exit normally!\n");
    else
        printf("program exit abnormally!\n");
    exit(s32Ret);

}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
