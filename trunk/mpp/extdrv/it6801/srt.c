/*

srt_func.c
Michael Liu

*/

#include <linux/string.h>
#include <linux/delay.h>
#undef abs //workaround:abs redefine
#include "srt.h"
#include "IO.h"


#define DEBUG_APPLIB_SRT
#if defined(DEBUG_APPLIB_SRT)
	#define DBGMSG printk
#else
	#define DBGMSG(...)
#endif

// #define true     1
// #define false    0
// typedef int bool;


unsigned char Encry(unsigned char udata)
{
	udata=((udata<<4) &0xf0) | ((udata >>4) &0x0f);	
	udata=((udata<<2) &0xcc) | ((udata >>2) &0x33);	
	udata=((udata<<1) &0xaa) | ((udata >>1) &0x55);	
	return udata;
}	

unsigned char Amba_app_srt_edesen_crypt(unsigned char *ck235_des_data,unsigned char *pDataBuffer)
{	
    unsigned char C[8]={0};
    unsigned char D[8]={0};    
    unsigned char E[8]={0};    
    unsigned char buf=0;
	unsigned char g_key[16];
	int i=0;
    g_key[0]=0xb3;
    g_key[1]=0xb4;
    g_key[2]=0xa0;
    g_key[3]=0x2d;
    g_key[4]=0xc6;
    g_key[5]=0x1e;
    g_key[6]=0xa5;
    g_key[7]=0xfe;
    g_key[8]=0x0b;
    g_key[9]=0x2d;
    g_key[10]=0x0a;
    g_key[11]=0x3f;
    g_key[12]=0x2e;
    g_key[13]=0xa8;
    g_key[14]=0x4b;
    g_key[15]=0x33;
	for(i=0;i<8;i++)
	{
		C[i]=(ck235_des_data[i]<<i) | (ck235_des_data[i+1]>>(8-i));
	}
	for(i=0;i<4;i++)
	{
		buf=C[i];
		C[i]=C[7-i];
		C[7-i]=buf;
	}
	for(i=0;i<8;i++)
	{
		C[i]=Encry(C[i]);
	}

	for(i=0;i<8;i++)
	{
		pDataBuffer[i]=C[i] ^ g_key[i] ^ g_key[i+8] ^ ck235_des_data[9];
		}
	if(ck235_des_data[0]!=ck235_des_data[1])
        {
	
	       
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	        for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[4] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]-E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[5] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[6] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	     for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[7] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
		}
             for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[7] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]+E[i];
	        }
	        for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[6] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
                 for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[5] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]-E[i];
	        }
	        for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[4] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[3] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[2] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]+E[i];
	        }
	     for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[1] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
			}
             for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[0] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	        for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[3] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]-E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[6] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	        for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[4] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[2] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]+E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[4] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	     for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[6] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
			}
             for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[3] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	        for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[2] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]-E[i];
	        }
                for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[0] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	        for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[1] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]+E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[7] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[5] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	     for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[5] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]-E[i];
			}
             for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[7] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	        for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[1] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]+E[i];
	        }
                 for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[0] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	        for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[2] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[5] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]-E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[1] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	     for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[7] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
			}
             for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[7] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]+E[i];
	        }
	        for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[4] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[3] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]-E[i];
	        }
	        for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[1] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[i] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[0] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	     for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[4] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] + E[i];
			}
             for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[i] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	        for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[1] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
                for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[6] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	        for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[6] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]+ E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[4] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	       for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[2] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
	     for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[1] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]-E[i];
			}
             for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[7] ^E[i];
	        }
	        for(i=0;i<8;i++) 
	        {
	        D[i]=D[i]-E[i];
	        }
	        for(i=0;i<8;i++)                                            
	        {
	        D[i]=pDataBuffer[i] ^E[i];
	        }
	           for(i=0;i<8;i++) 
	        {
	        D[i]=D[i] ^E[i];
	        }
          }
 return 0;
}







#define SRT_DEV_ADRR  0xc0
#define SRT_ADR_WRITE  0xc0
#define SRT_ADR_READ    0xc1


static int srt_idc_write(UINT16 Addr,UINT8 *pdata, int data_size)
{
	UINT8 offset = (UINT8)(Addr & 0x00ff);
	int ret = i2c_write_byte(SRT_DEV_ADRR, offset, data_size, pdata, 0);
	return ret;
}

static int srt_idc_read(UINT16 Addr, UINT8 *pdata, int data_size)
{
	UINT8 offset = (UINT8)(Addr & 0x00ff);
	int ret = i2c_read_byte(SRT_DEV_ADRR, offset, data_size, pdata, 0);
	return ret;
}

static void srt_msleep(unsigned int ms)
{
	msleep(ms);
}

int srt_check(void)
{
	//wite 8byte data
	UINT8 randata[8]={0x1A,0x2f,0x3f,0xff,0xEF,0xEA,0xAA,0x56};
	//write 10byte data
	UINT8 encdata[10]={0};
	//read  8byte data
	UINT8 decdata[8]={0};

	int i;
	int retry_i2c = 100;
	int result = 0;

	for(i=0; i < retry_i2c; i++)
	{
		result = srt_idc_write(0xA0,randata,8);
		if (result != 0)
		{
			continue;
		}
		srt_msleep(10);
		result = srt_idc_read(0xA0,encdata,10);
		if (result == 0)
		{
			break;
		}
	}

	if (result == 0)
	{
		DBGMSG("I2C success:i=%d\n", i);
		/*for(i = 0;i < 10; i++)
		{
			DBGMSG("encdata[%d]:0x%x\n",i,encdata[i]);
		}*/

		//dec data
		Amba_app_srt_edesen_crypt(encdata,decdata);

		//compare data
		for(i=0;i<8;i++)
		{
			if (decdata[i] != randata[i])
			{
				//rct_power_down();				
				result = -1;
			}

		    DBGMSG("randata[%d]==0x%x\n",i,randata[i]);
		    DBGMSG("decdata[%d]==0x%x\n",i,decdata[i]);
		}

		if (result == 0)
		{
			DBGMSG("compare success!!!\n");
			return 0;
		}
		else
		{
			DBGMSG("compare failed!!!\n");
			return -1;
		}
	}
	else
	{
		DBGMSG("I2C failed:i=%d\n",i);
		return -1;
	}
}

void AppLibCard_ConfigDefault(void)
{
	//wite 8byte data
	UINT8 randata[8]={0x1A,0x2f,0x3f,0xff,0xEF,0xEA,0xAA,0x56};
	//write 10byte data
	UINT8 encdata[10]={0};
	//read  8byte data
	UINT8 decdata[8]={0};

	int i;
	int retry_i2c = 100;
	int result = 0;

	for(i=0; i < retry_i2c; i++)
	{
		result = srt_idc_write(0xA0,randata,8);
		if (result != 0)
		{
			continue;
		}
		srt_msleep(10);
		result = srt_idc_read(0xA0,encdata,10);
		if (result == 0)
		{
			break;
		}
	}

	if (result == 0)
	{
		DBGMSG("I2C success:i=%d\n", i);
		/*for(i = 0;i < 10; i++)
		{
			DBGMSG("encdata[%d]:0x%x\n",i,encdata[i]);
		}*/

		//dec data
		Amba_app_srt_edesen_crypt(encdata,decdata);

		//compare data
		for(i=0;i<8;i++)
		{
			if (decdata[i] != randata[i])
			{
				//rct_power_down();				
				result = -1;
			}

		    DBGMSG("randata[%d]==0x%x\n",i,randata[i]);
		    DBGMSG("decdata[%d]==0x%x\n",i,decdata[i]);
		}

		if (result == 0)
		{
			DBGMSG("compare success!!!\n");
		}
		else
		{
			DBGMSG("compare failed!!!\n");
		}
	}
	else
	{
		DBGMSG("I2C failed:i=%d\n",i);
	}
	
	// assert(result == 0);
}

void AppLibVideoEnc_StartPipe(void)
{
	//wite 8byte data
	UINT8 randata[8]={0x1A,0x2f,0x3f,0xff,0xEF,0xEA,0xAA,0x56};
	//write 10byte data
	UINT8 encdata[10]={0};
	//read  8byte data
	UINT8 decdata[8]={0};

	int i;
	int retry_i2c = 100;
	int result = 0;

	for(i=0; i < retry_i2c; i++)
	{
		result = srt_idc_write(0xA0,randata,8);
		if (result != 0)
		{
			continue;
		}
		srt_msleep(4);
		result = srt_idc_read(0xA0,encdata,10);
		if (result == 0)
		{
			break;
		}
	}

	if (result == 0)
	{
		DBGMSG("I2C success:i=%d\n", i);
		/*for(i = 0;i < 10; i++)
		{
			DBGMSG("encdata[%d]:0x%x\n",i,encdata[i]);
		}*/

		//dec data
		Amba_app_srt_edesen_crypt(encdata,decdata);

		//compare data
		for(i=0;i<8;i++)
		{
			if (decdata[i] != randata[i])
			{
				//rct_power_down();				
				result = -1;
			}

		    DBGMSG("randata[%d]==0x%x\n",i,randata[i]);
		    DBGMSG("decdata[%d]==0x%x\n",i,decdata[i]);
		}

		if (result == 0)
		{
			DBGMSG("compare success!!!\n");
		}
		else
		{
			DBGMSG("compare failed!!!\n");
		}
	}
	else
	{
		DBGMSG("I2C failed:i=%d\n",i);
	}
	
	// assert(result == 0);
}

void AppLibVideoDec_Reset(void)
{
	//wite 8byte data
	UINT8 randata[8]={0x1A,0x2f,0x3f,0xff,0xEF,0xEA,0xAA,0x56};
	//write 10byte data
	UINT8 encdata[10]={0};
	//read  8byte data
	UINT8 decdata[8]={0};

	int i;
	int retry_i2c = 100;
	int result = 0;

	for(i=0; i < retry_i2c; i++)
	{
		result = srt_idc_write(0xA0,randata,8);
		if (result != 0)
		{
			continue;
		}
		srt_msleep(4);
		result = srt_idc_read(0xA0,encdata,10);
		if (result == 0)
		{
			break;
		}
	}

	if (result == 0)
	{
		DBGMSG("I2C success:i=%d\n", i);
		/*for(i = 0;i < 10; i++)
		{
			DBGMSG("encdata[%d]:0x%x\n",i,encdata[i]);
		}*/

		//dec data
		Amba_app_srt_edesen_crypt(encdata,decdata);

		//compare data
		for(i=0;i<8;i++)
		{
			if (decdata[i] != randata[i])
			{
				//rct_power_down();				
				result = -1;
			}

		    DBGMSG("randata[%d]==0x%x\n",i,randata[i]);
		    DBGMSG("decdata[%d]==0x%x\n",i,decdata[i]);
		}

		if (result == 0)
		{
			DBGMSG("compare success!!!\n");
		}
		else
		{
			DBGMSG("compare failed!!!\n");
		}
	}
	else
	{
		DBGMSG("I2C failed:i=%d\n",i);
	}
	
	// assert(result == 0);
}

/* EOF */


