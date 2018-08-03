///*****************************************
//  Copyright (C) 2009-2014
//  ITE Tech. Inc. All Rights Reserved
//  Proprietary and Confidential
///*****************************************
//   @file   <IO.c>
//   @author Max.Kao@ite.com.tw
//   @date   2014/06/26
//   @fileversion: ITE_MHLRX_SAMPLE_V1.10
//******************************************/
#include "config.h"
#include "IO.h"


#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/i2c.h>
//#include "nop.h"
//#include <stdio.h>

#define HDMI_DEV  0


//BYTE idata I2CDEV=0;

///////////////////////////////////////////////////////////////////////////////
//IO Pin Config For I2C control
//
//
//
//
///////////////////////////////////////////////////////////////////////////////

/*
#ifdef _IT6802_BOARD_
sbit  DEV0_SCL_PORT = P1^0;
sbit  DEV0_SDA_PORT = P1^1;


sbit  DEV1_SCL_PORT = P1^2;
sbit  DEV1_SDA_PORT = P1^3;

sbit  DEV2_SCL_PORT = P1^5;
sbit  DEV2_SDA_PORT = P1^6;

sbit  DEV3_SCL_PORT = P1^0;
sbit  DEV3_SDA_PORT = P1^1;

sbit  DEV4_SCL_PORT = P1^2;
sbit  DEV4_SDA_PORT = P1^3;
#else
sbit  DEV0_SCL_PORT = P0^0;
sbit  DEV0_SDA_PORT = P0^1;


sbit  DEV1_SCL_PORT = P0^2;
sbit  DEV1_SDA_PORT = P0^3;

sbit  DEV2_SCL_PORT = P0^5;
sbit  DEV2_SDA_PORT = P0^6;

sbit  DEV3_SCL_PORT = P0^0;
sbit  DEV3_SDA_PORT = P0^1;

sbit  DEV4_SCL_PORT = P0^2;
sbit  DEV4_SDA_PORT = P0^3;

#endif
*/

static struct i2c_board_info hdmi_info =
{
    I2C_BOARD_INFO("hdmi_i2c", 0x90),
};

static struct i2c_board_info edid_info =
{
    I2C_BOARD_INFO("edid_i2c", 0xA8),
};

static struct i2c_client* hdmi_client;
static struct i2c_client* edid_client;


////////////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////////////

void delay1ms(USHORT ms)
{
	mdelay(ms);
}

void HotPlug(BYTE bEnable)
{
	 //if(bEnable)
	 	//gpHPD0 = HPDON;
	 //else
	 	//gpHPD0 = HPDOFF;

}

void hdmi_read(BYTE offset,BYTE byteno,BYTE *p_data)
{
	BYTE i;
 	struct i2c_client* client = hdmi_client;
  unsigned char buf[2];
  int ret;
  
  //printk("i2c read\n");
 
	for(i = 0; i < byteno; ++i)
	{
		
    buf[0] = offset;
    ret = i2c_master_recv(client, buf, 1);
    if (ret >= 0)
    {
        p_data[i] = buf[0];
    }
		++offset;

	}

}

void hdmi_write(BYTE offset,BYTE byteno,BYTE *p_data)
{
	BYTE i;
 	struct i2c_client* client = hdmi_client;
  unsigned char buf[2];
  int ret=0;
  
  //printk("i2c write \n");
 
	
	for(i = 0; i < byteno; ++i)
	{
		buf[0] = offset + i;
    buf[1] = p_data[i];

    ret = i2c_master_send(client, buf, 2);
    if(ret !=2)
    {	
    	printk("i2c write failed\n");
    } 

	}
}

void edid_read(BYTE offset,BYTE byteno,BYTE *p_data)
{
	BYTE i;
 	struct i2c_client* client = edid_client;
  unsigned char buf[2];
  int ret;
  
  //printk("i2c edid read add\n");
 
  	
	for(i = 0; i < byteno; ++i)
	{
		
    buf[0] = offset;
    ret = i2c_master_recv(client, buf, 1);
    if (ret >= 0)
    {
        p_data[i] = buf[0];
    }
		++offset;

	}

}

void edid_write(BYTE offset,BYTE byteno,BYTE *p_data)
{
	BYTE i;
 	struct i2c_client* client = edid_client;
  unsigned char buf[2];
  int ret=0;
  
  //printk("i2c edid write\n");
 
	for(i = 0; i < byteno; ++i)
	{
		buf[0] = offset + i;
    buf[1] = p_data[i];

    ret = i2c_master_send(client, buf, 2);
    if(ret !=2)
    {	
    	printk("i2c write failed\n");
    
    } 

	}
}

SYS_STATUS i2c_write_byte( BYTE address,BYTE offset,BYTE byteno,BYTE *p_data,BYTE device )
{
 
 	if(address == 0x90)
 		hdmi_write(offset,byteno,p_data);
  else
  	edid_write(offset,byteno,p_data);

	return ER_SUCCESS;
}

SYS_STATUS i2c_read_byte( BYTE address,BYTE offset,BYTE byteno,BYTE *p_data,BYTE device )
{
	if(address == 0x90)
 		hdmi_read(offset,byteno,p_data);
  else
  	edid_read(offset,byteno,p_data);

  return ER_SUCCESS;
}


///////////////////////////////////////////////////////////////////////////////////////
//IIC control Functions
//
///////////////////////////////////////////////////////////////////////////////////////

#ifdef Enable_IT6802_CEC
BYTE IT6802_CEC_ReadI2C_Byte(BYTE RegAddr)
{
	 BYTE  p_data;
	 BOOL	FLAG;

	FLAG=i2c_read_byte(CEC_ADDR,RegAddr,1,&p_data,IT6802CECGPIOid);

	  if(FLAG==0)
	{
	 		CEC_DEBUG_PRINTF(("IT6802_CEC I2C ERROR !!!"));
			CEC_DEBUG_PRINTF(("=====  Read Reg0x%X=  \n",RegAddr));

	}

	 return p_data;
}


SYS_STATUS IT6802_CEC_WriteI2C_Byte(BYTE offset,BYTE buffer )
{
	 BOOL  flag;

	 flag=i2c_write_byte(CEC_ADDR,offset,1,&buffer,IT6802CECGPIOid);

	 return !flag;
}

#endif

/*

void w(unsigned char address, unsigned char data)
{
    //gpio_i2c_write(0x90, address, data);
    
    int ret;
    unsigned char buf[2];
    struct i2c_client* client = hdmi_client;

    buf[0] = address;
    buf[1] = data;

    ret = i2c_master_send(client, buf, 2);
    if(ret !=2)
    	printk("i2c write failed\n");
}

void w2(unsigned char address, unsigned char data)
{
    //gpio_i2c_write(0x90, address, data);
    
    int ret;
    unsigned char buf[2];
    struct i2c_client* client = hdmi_client;

    buf[0] = address;
    buf[1] = data;

    ret = i2c_master_send(client, buf, 2);
    if(ret !=2)
    	printk("i2c write failed\n");
}

unsigned char r(unsigned char address)
{
	  //unsigned char ret = gpio_i2c_read(0x90, address);
    
    unsigned char ret_data = 0xFF;
    int ret;
    struct i2c_client* client = edid_client;
    unsigned char buf[2];
    
 
    buf[0] = address;
    ret = i2c_master_recv(client, buf, 1);
    if (ret >= 0)
    {
        ret_data = buf[0];
    }
    
	  //printk("reg %x = %x\n", address, ret_data);
    return ret_data;
}

unsigned char r2(unsigned char address)
{
	  //unsigned char ret = gpio_i2c_read(0x90, address);
    
    unsigned char ret_data = 0xFF;
    int ret;
    struct i2c_client* client = edid_client;
    unsigned char buf[2];
    
 
    buf[0] = address;
    ret = i2c_master_recv(client, buf, 1);
    if (ret >= 0)
    {
        ret_data = buf[0];
    }
    
	  //printk("reg %x = %x\n", address, ret_data);
    return ret_data;
}
*/
void hi_dev_init(void)
{
    struct i2c_adapter* i2c_adap_0;
    struct i2c_adapter* i2c_adap_1;

    // use i2c0
    i2c_adap_0 = i2c_get_adapter(0);
    i2c_adap_1 = i2c_get_adapter(0);
    
    hdmi_client = i2c_new_device(i2c_adap_0, &hdmi_info);
    edid_client = i2c_new_device(i2c_adap_1, &edid_info);
		
		
    //printk(" hi_dev_init end\n");
		//printf("0x2 : 0x%x\n",r(0x2));
		//printf("0x3 : 0x%x\n",r(0x3));
		//printf("0x4 : 0x%x\n",r(0x4));
		
    //return 0;
}

void hi_dev_exit(void)
{
    //i2c_unregister_device(hdmi_client);
}

/*
BYTE HDMIRX_ReadI2C_Byte(BYTE RegAddr)
{
	return r(RegAddr);
}

SYS_STATUS HDMIRX_WriteI2C_Byte(BYTE RegAddr,BYTE val)
{
	w(RegAddr, val);
	return ER_SUCCESS;
}

SYS_STATUS HDMIRX_ReadI2C_ByteN(BYTE RegAddr,BYTE *pData,int N)
{
	int i;
	for(i = 0; i < N; ++i)
	{
		pData[i] = r(RegAddr);
		++RegAddr;
	}
    return ER_SUCCESS ;
}

SYS_STATUS HDMIRX_WriteI2C_ByteN(BYTE RegAddr,BYTE *pData,int N)
{
	int i = 0;
	for(i = 0; i < N; ++i)
	{
		w(RegAddr + i, pData[i]);
	}
	return ER_SUCCESS;
}
*/

