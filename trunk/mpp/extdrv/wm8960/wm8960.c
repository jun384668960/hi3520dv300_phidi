/* 
 *
 * Copyright (c) 2018 Hanztech Co., Ltd. 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA
 *
 * 
 * History: 
 *      2018-7-20 create this file
 */


#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/slab.h>
//#include <linux/smp_lock.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/system.h>
#ifndef CONFIG_HISI_SNAPSHOT_BOOT
#include <linux/miscdevice.h>
#endif
#include <linux/delay.h>

#include <linux/proc_fs.h>
#include <linux/poll.h>

#include <mach/hardware.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/irq.h>

#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
//#define HI_GPIO_I2C

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sound/wm8960.h>


#include "wm8960.h"
#include "wm8960_def.h"

#ifdef CONFIG_HISI_SNAPSHOT_BOOT
#include "himedia.h"
#endif

#define CHIP_NUM 1
#define DEV_NAME "wm8960"
#define DEBUG_LEVEL 1
#define DPRINTK(level,fmt,args...) do{ if(level < DEBUG_LEVEL)\
    printk(KERN_INFO "%s [%s ,%d]: " fmt "\n",DEV_NAME,__FUNCTION__,__LINE__,##args);\
}while(0)

unsigned int IIC_device_addr[CHIP_NUM] = {0x34};

static const struct reg_default wm8960_reg_defaults[] = {
	{  0x0, 0x00a7 },
	{  0x1, 0x00a7 },
	{  0x2, 0x0000 },
	{  0x3, 0x0000 },
	{  0x4, 0x0000 },
	{  0x5, 0x0008 },
	{  0x6, 0x0000 },
	{  0x7, 0x000a },
	{  0x8, 0x01c0 },
	{  0x9, 0x0000 },
	{  0xa, 0x00ff },
	{  0xb, 0x00ff },

	{ 0x10, 0x0000 },
	{ 0x11, 0x007b },
	{ 0x12, 0x0100 },
	{ 0x13, 0x0032 },
	{ 0x14, 0x0000 },
	{ 0x15, 0x00c3 },
	{ 0x16, 0x00c3 },
	{ 0x17, 0x01c0 },
	{ 0x18, 0x0000 },
	{ 0x19, 0x0000 },
	{ 0x1a, 0x0000 },
	{ 0x1b, 0x0000 },
	{ 0x1c, 0x0000 },
	{ 0x1d, 0x0000 },

	{ 0x20, 0x0100 },
	{ 0x21, 0x0100 },
	{ 0x22, 0x0050 },

	{ 0x25, 0x0050 },
	{ 0x26, 0x0000 },
	{ 0x27, 0x0000 },
	{ 0x28, 0x0000 },
	{ 0x29, 0x0000 },
	{ 0x2a, 0x0040 },
	{ 0x2b, 0x0000 },
	{ 0x2c, 0x0000 },
	{ 0x2d, 0x0050 },
	{ 0x2e, 0x0050 },
	{ 0x2f, 0x0000 },
	{ 0x30, 0x0002 },
	{ 0x31, 0x0037 },

	{ 0x33, 0x0080 },
	{ 0x34, 0x0008 },
	{ 0x35, 0x0031 },
	{ 0x36, 0x0026 },
	{ 0x37, 0x00e9 },
};

static struct i2c_board_info wm8960_info =
{
    I2C_BOARD_INFO("wm8960", 0x34),
};
static struct i2c_client* wm_client;
static unsigned int  open_cnt = 0;	
static int chip_count = 1;

#ifdef CONFIG_HISI_SNAPSHOT_BOOT
static struct himedia_device s_stWm8960Device;
#endif

static int wm8960_device_init(unsigned int num);
int wm8960_write(unsigned char chip_addr, unsigned int reg_addr, unsigned int value)
{
    if (reg_addr > MAX_REGISTER)
    {
        printk("%s:wrong reg address[reg_addr = 0x%x]\n",__FUNCTION__, reg_addr);
        return -1;   
    }
    else
    {    
        #define DATA_MASK (0x01ff)
        unsigned int data = value & DATA_MASK;
        unsigned char buf[2];

        buf[0] = (reg_addr << 1) | (data >> 8);
        buf[1] = (data & 0xff);

        struct i2c_client* client = wm_client;
        int ret = i2c_master_send(client, buf, 2);
        if (ret < 0) {
            printk("%s:error[reg_addr = 0x%x, value = 0x%x ret = %d]\n", \
                __FUNCTION__, reg_addr, value, ret);
        } else {
            printk("%s:success[reg_addr = 0x%x, value = 0x%x ret = %d]\n", \
                __FUNCTION__, reg_addr, value, ret);
    	}
        return ret;
    }
}

int wm8960_read(unsigned char chip_addr,unsigned char reg_addr)
{
    int ret_data = 0xFF;
/*    int ret;
    struct i2c_client* client = wm_client;
    unsigned char buf[2];

    buf[0] = reg_addr;
    ret = i2c_master_recv(client, buf, 1);
    if (ret >= 0)
    {
        ret_data = buf[0];
    }*/
    return ret_data;
}

void wm8960_reg_dump(unsigned int reg_num)
{
    unsigned int i = 0;
    for(i = 0;i < reg_num;i++)
    {
        printk("reg%d =%x,",i,wm8960_read(IIC_device_addr[0],i));
        if((i+1)%8==0)
        {
            printk("\n");
        }
    }
}
void soft_reset(unsigned int chip_num)
{
    wm8960_write(IIC_device_addr[chip_num], WM8960_RESET,       0x00);
    msleep(250);
    wm8960_write(IIC_device_addr[chip_num], WM8960_POWER1,      0xfc);
    msleep(250);
    wm8960_write(IIC_device_addr[chip_num], WM8960_PLL1,        0x37);
    wm8960_write(IIC_device_addr[chip_num], WM8960_PLL2,        0x86);
    wm8960_write(IIC_device_addr[chip_num], WM8960_PLL3,        0xc2);
    wm8960_write(IIC_device_addr[chip_num], WM8960_PLL4,        0x26);
    wm8960_write(IIC_device_addr[chip_num], WM8960_CLOCK1,      0x5);
    wm8960_write(IIC_device_addr[chip_num], WM8960_CLOCK2,      0x1c4);
    wm8960_write(IIC_device_addr[chip_num], WM8960_IFACE1,      0x42);
    wm8960_write(IIC_device_addr[chip_num], WM8960_POWER2,      0x1fb);
    msleep(250);
    wm8960_write(IIC_device_addr[chip_num], WM8960_POWER3,      0x3c);
    wm8960_write(IIC_device_addr[chip_num], WM8960_LINVOL,      0x13f);
    wm8960_write(IIC_device_addr[chip_num], WM8960_RINVOL,      0x13f);
    wm8960_write(IIC_device_addr[chip_num], WM8960_LOUT1,       0x17f);
    wm8960_write(IIC_device_addr[chip_num], WM8960_ROUT1,       0x17f);
    wm8960_write(IIC_device_addr[chip_num], WM8960_DACCTL1,     0x0);
    wm8960_write(IIC_device_addr[chip_num], WM8960_IFACE2,      0x40);
    wm8960_write(IIC_device_addr[chip_num], WM8960_LDAC,        0xff);
    wm8960_write(IIC_device_addr[chip_num], WM8960_RDAC,        0xff);
    wm8960_write(IIC_device_addr[chip_num], WM8960_ALC2,        0x0);
    wm8960_write(IIC_device_addr[chip_num], WM8960_LADC,        0xc3);
    wm8960_write(IIC_device_addr[chip_num], WM8960_RADC,        0xc3);
    wm8960_write(IIC_device_addr[chip_num], WM8960_APOP1,       0x8);
    wm8960_write(IIC_device_addr[chip_num], WM8960_LINPATH,     0x108);
    wm8960_write(IIC_device_addr[chip_num], WM8960_RINPATH,     0x108);
    wm8960_write(IIC_device_addr[chip_num], WM8960_LOUTMIX,     0x100);
    wm8960_write(IIC_device_addr[chip_num], WM8960_ROUTMIX,     0x100);
    wm8960_write(IIC_device_addr[chip_num], WM8960_LOUT2,       0x179);
    wm8960_write(IIC_device_addr[chip_num], WM8960_ROUT2,       0x179);
    wm8960_write(IIC_device_addr[chip_num], WM8960_ADDCTL4,     0x3);
    wm8960_write(IIC_device_addr[chip_num], WM8960_CLASSD1,     0xf7);
    wm8960_write(IIC_device_addr[chip_num], WM8960_CLASSD3,     0x9b);
}        	

/*
 *	device open. set counter
 */
static int wm8960_open(struct inode * inode, struct file * file)
{
	if(0 == open_cnt++)
		return 0;    	
	return -1 ;
}

/*
 *	Close device, Do nothing!
 */
static int wm8960_close(struct inode *inode ,struct file *file)
{
	open_cnt--;
    	return 0;
}

//static int wm8960_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
static long wm8960_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned int __user *argp = (unsigned int __user *)arg;
    unsigned int chip_num;
	Audio_Ctrl temp;
    Audio_Ctrl *audio_ctrl;
    Codec_Datapath_Setup_Ctrl codec_datapath_setup_ctrl;
    DAC_OUTPUT_SWIT_CTRL dac_output_swit_ctrl;
    DAC_POWER_CTRL dac_power_ctrl;
    In1_Adc_Ctrl in1_adc_ctrl ;
    In2_Adc_Ctrl_Sample in2_adc_ctrl_sample ;
    Adc_Pga_Dac_Gain_Ctrl adc_pga_dac_gain_ctrl;
    Line_Hpcom_Out_Ctrl line_hpcom_out_ctrl;
    Serial_Int_Ctrl serial_int_ctrl;
    Serial_Data_Offset_Ctrl serial_data_offset_ctrl;
    Ctrl_Mode ctrl_mode; 

	if(argp != NULL)
	{
        if(copy_from_user(&temp,argp,sizeof(Audio_Ctrl)))
	    {   
    	    return -EFAULT;
   	 	}
	}
    audio_ctrl = (Audio_Ctrl *)(&temp);
    chip_num = audio_ctrl->chip_num;
    switch(cmd)
    {
        case IN2LR_2_LEFT_ADC_CTRL:
            in2_adc_ctrl_sample.b8 = wm8960_read(IIC_device_addr[chip_num],17);      
            in2_adc_ctrl_sample.bit.in2l_adc_input_level_sample = audio_ctrl->input_level;
            wm8960_write(IIC_device_addr[chip_num],17,in2_adc_ctrl_sample.b8);
            break;
        case IN2LR_2_RIGTH_ADC_CTRL:
            in2_adc_ctrl_sample.b8 = wm8960_read(IIC_device_addr[chip_num],18);      
            in2_adc_ctrl_sample.bit.in2r_adc_input_level_sample = audio_ctrl->input_level;
            wm8960_write(IIC_device_addr[chip_num],18,in2_adc_ctrl_sample.b8);
             
            break;
        case IN1L_2_LEFT_ADC_CTRL:
            in1_adc_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],19);      
            in1_adc_ctrl.bit.in1_adc_input_level = audio_ctrl->input_level;
            in1_adc_ctrl.bit.adc_ch_power_ctrl = audio_ctrl->if_powerup;
            wm8960_write(IIC_device_addr[chip_num],19,in1_adc_ctrl.b8);
            break;
        case IN1R_2_RIGHT_ADC_CTRL:
            in1_adc_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],22);      
            in1_adc_ctrl.bit.in1_adc_input_level = audio_ctrl->input_level;
            in1_adc_ctrl.bit.adc_ch_power_ctrl = audio_ctrl->if_powerup;
            wm8960_write(IIC_device_addr[chip_num],22,in1_adc_ctrl.b8);
            break;
        case PGAL_2_HPLOUT_VOL_CTRL:
            adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],46);
            adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
            adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
            wm8960_write(IIC_device_addr[chip_num],46,adc_pga_dac_gain_ctrl.b8);
            break;
        case DACL1_2_HPLOUT_VOL_CTRL:
            adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],47);
            adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
            adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
            wm8960_write(IIC_device_addr[chip_num],47,adc_pga_dac_gain_ctrl.b8);
            break;
        case HPLOUT_OUTPUT_LEVEL_CTRL:
            line_hpcom_out_ctrl.b8 =  wm8960_read(IIC_device_addr[chip_num],51); 
            line_hpcom_out_ctrl.bit.if_mute = audio_ctrl->if_mute_route;
            line_hpcom_out_ctrl.bit.output_level = audio_ctrl->input_level;
			line_hpcom_out_ctrl.bit.power_status = audio_ctrl->if_powerup;
            wm8960_write(IIC_device_addr[chip_num],51,line_hpcom_out_ctrl.b8);
            break; 
        case PGAL_2_HPLCOM_VOL_CTRL:
            adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],53);
            adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
            adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
            wm8960_write(IIC_device_addr[chip_num],53,adc_pga_dac_gain_ctrl.b8);
            break;
        case DACL1_2_HPLCOM_VOL_CTRL:
            adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],54);
            adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
            adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
            wm8960_write(IIC_device_addr[chip_num],54,adc_pga_dac_gain_ctrl.b8);
            break;
        case HPLCOM_OUTPUT_LEVEL_CTRL:
           line_hpcom_out_ctrl.b8 =  wm8960_read(IIC_device_addr[chip_num],58); 
           line_hpcom_out_ctrl.bit.if_mute = audio_ctrl->if_mute_route;
           line_hpcom_out_ctrl.bit.output_level =  audio_ctrl->input_level;
           wm8960_write(IIC_device_addr[chip_num],58,line_hpcom_out_ctrl.b8);
          break;
        case PGAR_2_HPROUT_VOL_CTRL:
            adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],63);
            adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
            adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
            wm8960_write(IIC_device_addr[chip_num],63,adc_pga_dac_gain_ctrl.b8);
            break;
        case DACR1_2_HPROUT_VOL_CTRL: 
            adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],64);
            adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
            adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
            wm8960_write(IIC_device_addr[chip_num],64,adc_pga_dac_gain_ctrl.b8);
            break;
        case HPROUT_OUTPUT_LEVEL_CTRL:
           line_hpcom_out_ctrl.b8 =  wm8960_read(IIC_device_addr[chip_num],65); 
           line_hpcom_out_ctrl.bit.if_mute = audio_ctrl->if_mute_route;
           line_hpcom_out_ctrl.bit.output_level = audio_ctrl->input_level;
		   line_hpcom_out_ctrl.bit.power_status = audio_ctrl->if_powerup;
           wm8960_write(IIC_device_addr[chip_num],65,line_hpcom_out_ctrl.b8);
           break;
        case PGAR_2_HPRCOM_VOL_CTRL: 
              adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],70);
              adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
              adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
              wm8960_write(IIC_device_addr[chip_num],70,adc_pga_dac_gain_ctrl.b8);
              break;
        case DACR1_2_HPRCOM_VOL_CTRL:
             adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],71);
              adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route; 
              adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
              wm8960_write(IIC_device_addr[chip_num],71,adc_pga_dac_gain_ctrl.b8);
                break;
        case HPRCOM_OUTPUT_LEVEL_CTRL:
              line_hpcom_out_ctrl.b8 =  wm8960_read(IIC_device_addr[chip_num],72); 
               line_hpcom_out_ctrl.bit.if_mute = audio_ctrl->if_mute_route;
               line_hpcom_out_ctrl.bit.output_level =  audio_ctrl->input_level;
               wm8960_write(IIC_device_addr[chip_num],72,line_hpcom_out_ctrl.b8);
              break;
        case PGAL_2_LEFT_LOP_VOL_CTRL:
              adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],81);
              adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
              adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
              wm8960_write(IIC_device_addr[chip_num],81,adc_pga_dac_gain_ctrl.b8);
                break;
        case DACL1_2_LEFT_LOP_VOL_CTRL:
             adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],82);
              adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route; 
              adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
              wm8960_write(IIC_device_addr[chip_num],82,adc_pga_dac_gain_ctrl.b8);
                break;
        case LEFT_LOP_OUTPUT_LEVEL_CTRL:
              line_hpcom_out_ctrl.b8 =  wm8960_read(IIC_device_addr[chip_num],86); 
               line_hpcom_out_ctrl.bit.if_mute = audio_ctrl->if_mute_route;
               line_hpcom_out_ctrl.bit.output_level =  audio_ctrl->input_level;
               line_hpcom_out_ctrl.bit.power_status =  audio_ctrl->if_powerup;
               wm8960_write(IIC_device_addr[chip_num],86,line_hpcom_out_ctrl.b8);
              break;
        case PGAR_2_RIGHT_LOP_VOL_CTRL:
              adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],91);
              adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
              adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
              wm8960_write(IIC_device_addr[chip_num],91,adc_pga_dac_gain_ctrl.b8);
                break;
        case DACR1_2_RIGHT_LOP_VOL_CTRL:
             adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],92);
              adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route; 
              adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
              wm8960_write(IIC_device_addr[chip_num],92,adc_pga_dac_gain_ctrl.b8);
                break;
        case RIGHT_LOP_OUTPUT_LEVEL_CTRL:
              line_hpcom_out_ctrl.b8 =  wm8960_read(IIC_device_addr[chip_num],93); 
               line_hpcom_out_ctrl.bit.if_mute = audio_ctrl->if_mute_route;
               line_hpcom_out_ctrl.bit.output_level =  audio_ctrl->input_level;
               line_hpcom_out_ctrl.bit.power_status =  audio_ctrl->if_powerup;
               wm8960_write(IIC_device_addr[chip_num],93,line_hpcom_out_ctrl.b8);
              break;
        case SET_ADC_SAMPLE:
                in2_adc_ctrl_sample.b8 = wm8960_read(IIC_device_addr[chip_num],2);      
                in2_adc_ctrl_sample.bit.in2l_adc_input_level_sample = audio_ctrl->sample;
                wm8960_write(IIC_device_addr[chip_num],2,in2_adc_ctrl_sample.b8);
                break;
        case SET_DAC_SAMPLE:
                in2_adc_ctrl_sample.b8 = wm8960_read(IIC_device_addr[chip_num],2);      
                in2_adc_ctrl_sample.bit.in2r_adc_input_level_sample = audio_ctrl->sample;
                wm8960_write(IIC_device_addr[chip_num],2,in2_adc_ctrl_sample.b8);
                //printk("set SET_DAC_SAMPLE,audio_ctrl->sample=%x\n",audio_ctrl->sample);
                break;
        case SET_DATA_LENGTH:
                serial_int_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],9);;                
                serial_int_ctrl.bit.data_length = audio_ctrl->data_length;
                //wm8960_write(IIC_device_addr[chip_num],9,serial_int_ctrl.b8);
                break;
        case SET_TRANSFER_MODE:
                serial_int_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],9);    
                serial_int_ctrl.bit.transfer_mode = audio_ctrl->trans_mode;
                wm8960_write(IIC_device_addr[chip_num],9,serial_int_ctrl.b8);
                break;              
        case SET_CTRL_MODE:
                //wm8960_write(IIC_device_addr[chip_num],0x1,0x80);
                //udelay(50);  
                ctrl_mode.b8 = wm8960_read(IIC_device_addr[chip_num],8);
                ctrl_mode.bit.bit_clock_dic_ctrl =  audio_ctrl->ctrl_mode;
                ctrl_mode.bit.work_clock_dic_ctrl =  audio_ctrl->ctrl_mode;
                ctrl_mode.bit.bit_work_dri_ctrl =  audio_ctrl->ctrl_mode;
                wm8960_write(IIC_device_addr[chip_num],8,ctrl_mode.b8);
#if 0
                /* 设置时钟 */
                if (1 == audio_ctrl->ctrl_mode 
                    || (AC31_SET_48K_SAMPLERATE != audio_ctrl->sample && AC31_SET_44_1K_SAMPLERATE != audio_ctrl->sample))
                {
                    /* aic31作主模式或者采样率不为44.1K/48KHZ的情况下，使用外部的12.288MHZ的晶振作为MCLK输入并产生内部工作主时钟 */
                    if ((1 == audio_ctrl->if_44100hz_series))
                    {
                        /*　如果为44.1KHZ系列的采样样 */
                        wm8960_write(IIC_device_addr[chip_num],3,0x81);    /* P=1 */ 
                        wm8960_write(IIC_device_addr[chip_num],4,0x1c);    /* J=7 */
                        wm8960_write(IIC_device_addr[chip_num],5,0x36);    /* reg 5 and 6 set D=3500*/
                        wm8960_write(IIC_device_addr[chip_num],6,0xb0);
                        codec_datapath_setup_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],7);
                        codec_datapath_setup_ctrl.b8 |= 0x80;   /* FSref = 44.1 kHz */
                        wm8960_write(IIC_device_addr[chip_num],7,codec_datapath_setup_ctrl.b8);
                        wm8960_write(IIC_device_addr[chip_num],11,0x1);    /* R=1 */
                        wm8960_write(IIC_device_addr[chip_num],101,0x0);
                        wm8960_write(IIC_device_addr[chip_num],102,0xc2);
                    }
                    else
                    {
                        /*　如果为非44.1KHZ系列的采样样 */
                        wm8960_write(IIC_device_addr[chip_num],3,0x81);    /* P=1 */ 
                        wm8960_write(IIC_device_addr[chip_num],4,0x20);    /* J=8 */
                        wm8960_write(IIC_device_addr[chip_num],5,0x0);     /* reg 5 and 6 set D=0000*/
                        wm8960_write(IIC_device_addr[chip_num],6,0x0);
                        codec_datapath_setup_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],7);
                        codec_datapath_setup_ctrl.b8 &= 0x7f;   /* FSref = 48 kHz */
                        wm8960_write(IIC_device_addr[chip_num],7,codec_datapath_setup_ctrl.b8);
                        wm8960_write(IIC_device_addr[chip_num],11,0x1);    /* R=1 */
                        wm8960_write(IIC_device_addr[chip_num],101,0x0);
                        wm8960_write(IIC_device_addr[chip_num],102,0xc2);
                    }
                }
                else
                {
                    /* aic31做从模式且采样率为44.1K/48KHZ的情况下，由BCLK产生内部工作主时钟 */
                    wm8960_write(IIC_device_addr[chip_num],102,0x22);  /* uses PLLCLK and BCLK */
                    codec_datapath_setup_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],7);
                    if ((1 == audio_ctrl->if_44100hz_series))
                    {
                        codec_datapath_setup_ctrl.b8 |= 0x80;   /* FSref = 44.1 kHz */
                    }
                    else
                    {
                        codec_datapath_setup_ctrl.b8 &= 0x7f;   /* FSref = 48 kHz */
                    }
                    wm8960_write(IIC_device_addr[chip_num],7,codec_datapath_setup_ctrl.b8);

                    wm8960_write(IIC_device_addr[chip_num],3,0x81);    /* P=1 */ 
                    wm8960_write(IIC_device_addr[chip_num],4,32<<2);   /* set PLL J to 32 */
                    wm8960_write(IIC_device_addr[chip_num],5,0x0);     /* reg 5 and 6 set D=0000*/
                    wm8960_write(IIC_device_addr[chip_num],6,0x0);
                    wm8960_write(IIC_device_addr[chip_num],101,0x0);   /* CODEC_CLKIN uses PLLDIV_OUT */
                    wm8960_write(IIC_device_addr[chip_num],11,0x2);    /* R = 2 */
                }
#else
 				/* 设置时钟 */
               /* aic31,由aiao提供mclk */             
          		switch(audio_ctrl->sampleRate)
            	{
            		case 8000:
            		case 16000:
            		case 32000:
            			{
            			    /*　如果为32KHZ系列的采样样 */
	                        wm8960_write(IIC_device_addr[chip_num],3,0x81);    /* P=1 */ 
	                        wm8960_write(IIC_device_addr[chip_num],4,0x30);    /* J=12 */
	                        wm8960_write(IIC_device_addr[chip_num],5,0x0);     /* reg 5 and 6 set D=0000*/
	                        wm8960_write(IIC_device_addr[chip_num],6,0x0);
	                        codec_datapath_setup_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],7);
	                        codec_datapath_setup_ctrl.b8 &= 0x7f;   /* FSref = 48 kHz */
	                        wm8960_write(IIC_device_addr[chip_num],7,codec_datapath_setup_ctrl.b8);
	                        wm8960_write(IIC_device_addr[chip_num],11,0x1);    /* R=1 */
	                        wm8960_write(IIC_device_addr[chip_num],101,0x0);
	                        wm8960_write(IIC_device_addr[chip_num],102,0xc2);
            			}
            			break;
            		case 12000:
            		case 24000:
            		case 48000:                		
            		    {
            		        /*　如果为48KHZ系列的采样样 */
	                        wm8960_write(IIC_device_addr[chip_num],3,0x81);    /* P=1 */ 
	                        wm8960_write(IIC_device_addr[chip_num],4,0x20);    /* J=8 */
	                        wm8960_write(IIC_device_addr[chip_num],5,0x0);     /* reg 5 and 6 set D=0000*/
	                        wm8960_write(IIC_device_addr[chip_num],6,0x0);
	                        codec_datapath_setup_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],7);
	                        codec_datapath_setup_ctrl.b8 &= 0x7f;   /* FSref = 48 kHz */
	                        wm8960_write(IIC_device_addr[chip_num],7,codec_datapath_setup_ctrl.b8);
	                        wm8960_write(IIC_device_addr[chip_num],11,0x1);    /* R=1 */
	                        wm8960_write(IIC_device_addr[chip_num],101,0x0);
	                        wm8960_write(IIC_device_addr[chip_num],102,0xc2);
                        }
            			break;
            		case 11025:
            		case 22050:
            		case 44100:
            		    {
                		    /*　如果为44.1KHZ系列的采样样 */
	                        wm8960_write(IIC_device_addr[chip_num],3,0x81);    /* P=1 */ 
	                        wm8960_write(IIC_device_addr[chip_num],4,0x20);    /* J=7 */
	                        wm8960_write(IIC_device_addr[chip_num],5,0x00);    /* reg 5 and 6 set D=0000*/
	                        wm8960_write(IIC_device_addr[chip_num],6,0x00);
	                        codec_datapath_setup_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],7);
	                        codec_datapath_setup_ctrl.b8 |= 0x80;   /* FSref = 44.1 kHz */
	                        wm8960_write(IIC_device_addr[chip_num],7,codec_datapath_setup_ctrl.b8);
	                        wm8960_write(IIC_device_addr[chip_num],11,0x1);    /* R=1 */
	                        wm8960_write(IIC_device_addr[chip_num],101,0x0);
	                        wm8960_write(IIC_device_addr[chip_num],102,0xc2);
            		    }
            			break;               			
                
                default:
                	printk("aic31 unsupport sampleRate %d\n", audio_ctrl->sampleRate);
                	return -1;
            	}	
               
#endif
                break;
        case LEFT_DAC_VOL_CTRL:
                adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],43);
                adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route; 
                adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
                wm8960_write(IIC_device_addr[chip_num],43,adc_pga_dac_gain_ctrl.b8);
                break;
        case RIGHT_DAC_VOL_CTRL:
                adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],44);
                adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route; 
                adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
                wm8960_write(IIC_device_addr[chip_num],44,adc_pga_dac_gain_ctrl.b8);
                break;
        case LEFT_DAC_POWER_SETUP:
                codec_datapath_setup_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],7);
                codec_datapath_setup_ctrl.bit.left_dac_datapath_ctrl = audio_ctrl->if_powerup;
                wm8960_write(IIC_device_addr[chip_num],7,codec_datapath_setup_ctrl.b8);
                dac_power_ctrl.b8 =  wm8960_read(IIC_device_addr[chip_num],37);
                dac_power_ctrl.bit.left_dac_power_ctrl =  audio_ctrl->if_powerup;
                wm8960_write(IIC_device_addr[chip_num],37,dac_power_ctrl.b8);
                 break;
        case RIGHT_DAC_POWER_SETUP:
                codec_datapath_setup_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],7);
                codec_datapath_setup_ctrl.bit.right_dac_datapath_ctrl = audio_ctrl->if_powerup;
                wm8960_write(IIC_device_addr[chip_num],7,codec_datapath_setup_ctrl.b8);
                dac_power_ctrl.b8 =  wm8960_read(IIC_device_addr[chip_num],37);
                dac_power_ctrl.bit.right_dac_power_ctrl =  audio_ctrl->if_powerup;
                wm8960_write(IIC_device_addr[chip_num],37,dac_power_ctrl.b8);
                 break;
        case DAC_OUT_SWITCH_CTRL:  
                dac_output_swit_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],41);
                dac_output_swit_ctrl.bit.left_dac_swi_ctrl =  audio_ctrl->dac_path;
                dac_output_swit_ctrl.bit.right_dac_swi_ctrl = audio_ctrl->dac_path;
                wm8960_write(IIC_device_addr[chip_num],41,dac_output_swit_ctrl.b8);
                 break;
        case LEFT_ADC_PGA_CTRL:
                adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],15);
                adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
                adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
                wm8960_write(IIC_device_addr[chip_num],15,adc_pga_dac_gain_ctrl.b8);
                break;
        case RIGHT_ADC_PGA_CTRL:
                adc_pga_dac_gain_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],16);
                adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
                adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
                wm8960_write(IIC_device_addr[chip_num],16,adc_pga_dac_gain_ctrl.b8);
                break;
        case SET_SERIAL_DATA_OFFSET:
                serial_data_offset_ctrl.b8 = wm8960_read(IIC_device_addr[chip_num],10);
                serial_data_offset_ctrl.bit.serial_data_offset = audio_ctrl->data_offset;
                wm8960_write(IIC_device_addr[chip_num],10,serial_data_offset_ctrl.b8);
                break;
        case SOFT_RESET:
                //printk("[Func]:%s [Line]:%d [Info]:%s\n", __FUNCTION__, __LINE__, "invalid attribute");
                soft_reset(chip_num); 
                break;
        case WM8960_REG_DUMP:
                wm8960_reg_dump(102);
                break;
        default:
                break;
    }
    return 0;
}

#ifdef CONFIG_HISI_SNAPSHOT_BOOT
static int wm8960_freeze(struct himedia_device* pdev)
{
    printk(KERN_ALERT "%s  %d\n", __FUNCTION__, __LINE__);
    return 0;
}
static int wm8960_restore(struct himedia_device* pdev)
{
    int i;
    for (i = 0; i < chip_count; i++)
    {
        if (wm8960_device_init(i) < 0)
        {
            printk(KERN_ALERT "%s  %d, wm8960 device init fail!\n", __FUNCTION__, __LINE__);
            return -1;
        }
    }
    printk(KERN_ALERT "%s  %d\n", __FUNCTION__, __LINE__);
    return 0;
}
#endif
/*
 *  The various file operations we support.
 */
 
static struct file_operations wm8960_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl		= wm8960_ioctl,
	.open		= wm8960_open,
	.release	= wm8960_close
};

#ifdef CONFIG_HISI_SNAPSHOT_BOOT
struct himedia_ops stWm8960DrvOps =
{
    .pm_freeze = wm8960_freeze,
    .pm_restore  = wm8960_restore
};
#else
static struct miscdevice wm8960_dev =
{
	MISC_DYNAMIC_MINOR,
	DEV_NAME,
	&wm8960_fops,
};
#endif

static int set_chip_count(const char* val, const struct kernel_param* kp)
{
    int ret;
    int chip_count;
    ret = kstrtoint(val, 10, &chip_count);
    if (ret < 0)
    {
        return -EINVAL;
    }
    if (chip_count < 0 || chip_count > CHIP_NUM)
    {
        printk("chip_count%d err. \n", chip_count);
        return -EINVAL;
    }
    return 0;
}
static struct kernel_param_ops wm8960_para_ops =
{
    .set = set_chip_count,
};
#if 0
module_param(chip_count, int, 0);
#else
module_param_cb(chip_count, &wm8960_para_ops, &chip_count, 0644);
#endif
MODULE_PARM_DESC(chip_count, "the num we device uses the wm8960,default 1");
static int wm8960_reboot(struct notifier_block* self, unsigned long data, void* pdata)
{
    unsigned int i;
    for (i = 0; i < chip_count; i++)
    {
        wm8960_write(IIC_device_addr[i], 51, 0x04);
        wm8960_write(IIC_device_addr[i], 65, 0x04);
    }
    printk("Func:%s, line:%d######\n", __FUNCTION__, __LINE__);
    return 0;
}
static struct notifier_block wm8960_reboot_notifier =
{
    .notifier_call = wm8960_reboot,
};
static int wm8960_device_init(unsigned int num)
{
    soft_reset(num);

    #if 0 // reference to kernel/linux-3.10.y/sound/soc/codecs/wm8960.c
    int ret;
    unsigned int reg_addr, value;
    unsigned int reg_num = sizeof(wm8960_reg_defaults) / sizeof(struct reg_default);
    for (int i = 0; i < reg_num; ++i)
    {   
        reg_addr = wm8960_reg_defaults[i].reg;
        value    = wm8960_reg_defaults[i].def;
        ret = wm8960_write(IIC_device_addr[num], reg_addr, value);
        if (ret < 0) {
            printk("%s:error[reg_addr = 0x%x, value = 0x%x ret = %d]\n", \
                __FUNCTION__, reg_addr, value, ret);
        } else {
            printk("%s:success[reg_addr = 0x%x, value = 0x%x ret = %d]\n", \
                __FUNCTION__, reg_addr, value, ret);
        }
    }
    #endif

    // register_reboot_notifier(&wm8960_reboot_notifier);
    
    return 0;
}
static int wm8960_device_exit(unsigned int num)
        {
    // wm8960_write(IIC_device_addr[num], 51, 0x04);

    // wm8960_write(IIC_device_addr[num], 65, 0x04);

    	return 0;
}  

static int i2c_client_init(void)
{
    struct i2c_adapter* i2c_adap;

    // use i2c0
    i2c_adap = i2c_get_adapter(0);
    wm_client = i2c_new_device(i2c_adap, &wm8960_info);

    i2c_put_adapter(i2c_adap);

	return 0;
}

static void i2c_client_exit(void)
{
    i2c_unregister_device(wm_client);
}

static int __init wm8960_init(void)
{
    	unsigned int i,ret;

#ifdef CONFIG_HISI_SNAPSHOT_BOOT
    snprintf(s_stWm8960Device.devfs_name, sizeof(s_stWm8960Device.devfs_name), DEV_NAME);
    s_stWm8960Device.minor  = HIMEDIA_DYNAMIC_MINOR;
    s_stWm8960Device.fops   = &wm8960_fops;
    s_stWm8960Device.drvops = &stWm8960DrvOps;
    s_stWm8960Device.owner  = THIS_MODULE;
    ret = himedia_register(&s_stWm8960Device);
    if (ret)
    {
        DPRINTK(0, "could not register wm8960 device");
        return -1;
    }
#else
    	ret = misc_register(&wm8960_dev);
    	if(ret < 0)
    	{
    		DPRINTK(0,"could not register wm8960 device");
    		return -1;
    	}
#endif
        i2c_client_init();
        for(i = 0;i< chip_count;i++)
        {
            if(wm8960_device_init(i) < 0)
            {
                goto init_fail;
            }
        }
    	DPRINTK(1,"wm8960 driver init successful!");
    printk("load wm8960.ko  ok!\n");
    	return ret;
init_fail:
#ifdef CONFIG_HISI_SNAPSHOT_BOOT
    himedia_unregister(&s_stWm8960Device);
#else
        misc_deregister(&wm8960_dev);
#endif
        DPRINTK(0,"wm8960 device init fail,deregister it!");
        return -1;
}     

static void __exit wm8960_exit(void)
{
    unsigned int i;
    for (i = 0; i < chip_count; i++)
    {
        wm8960_device_exit(i);
    }
    unregister_reboot_notifier(&wm8960_reboot_notifier);
#ifdef CONFIG_HISI_SNAPSHOT_BOOT
    himedia_unregister(&s_stWm8960Device);
#else
    misc_deregister(&wm8960_dev);
#endif

    i2c_client_exit();
    DPRINTK(1,"deregister wm8960");
    printk("rmmod wm8960.ko  ok!\n");
}

module_init(wm8960_init);
module_exit(wm8960_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hanztech");

