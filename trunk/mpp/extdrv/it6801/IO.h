///*****************************************
//  Copyright (C) 2009-2014
//  ITE Tech. Inc. All Rights Reserved
//  Proprietary and Confidential
///*****************************************
//   @file   <IO.h>
//   @author Max.Kao@ite.com.tw
//   @date   2014/06/26
//   @fileversion: ITE_MHLRX_SAMPLE_V1.10
//******************************************/

#ifndef _IO_h_
#define _IO_h_
#include "typedef.h"


#ifdef SUPPORT_UART_CMD
void UartCommand();
#endif


//BYTE HDMI_IIC_Read( BYTE RegAddr);
//BOOL HDMI_IIC_Write( BYTE RegAddr,BYTE DataIn);
//BYTE HDMI_IIC_SET( BYTE offset, BYTE mask, BYTE datain );

void delay1ms(USHORT ms);

SYS_STATUS i2c_write_byte( BYTE address,BYTE offset,BYTE byteno,BYTE *p_data,BYTE device );
SYS_STATUS i2c_read_byte( BYTE address,BYTE offset,BYTE byteno,BYTE *p_data,BYTE device );
void hi_dev_init(void);

//#define hdmirxrd(x) HDMI_IIC_Read(x)
//#define hdmirxwr(x,y) HDMI_IIC_Write(x,y)



//void SetEDIDWp(BOOL ENA);


void HotPlug(BYTE Enable);

void init_printf(void);

//void SetintActive(BOOL bactive);


#endif
