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


#include "wm_8960.h"
#include "wm_8960_def.h"

#ifdef CONFIG_HISI_SNAPSHOT_BOOT
#include "himedia.h"
#endif

#define CHIP_NUM 1
#define DEV_NAME "wm8960"
#define DEBUG_LEVEL 1
#define DPRINTK(level,fmt,args...) do{ if(level < DEBUG_LEVEL)\
    printk(KERN_INFO "%s [%s ,%d]: " fmt "\n",DEV_NAME,__FUNCTION__,__LINE__,##args);\
}while(0)

static unsigned int IIC_device_addr[CHIP_NUM] = {0x34};

static struct reg_default wm_8960_reg_defaults[] = {
	{  0x0, 0x0097 },
	{  0x1, 0x0097 },
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
int wm8960_write(unsigned int reg_addr, unsigned int value)
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
            printk("%s:error[reg_addr = 0x%02x, value = 0x%3x ret = %d]\n", \
                __FUNCTION__, reg_addr, value, ret);
        } else {
            printk("%s:success[reg_addr = 0x%02x, value = 0x%3x ret = %d]\n", \
                __FUNCTION__, reg_addr, value, ret);
    	}
        return ret;
    }
}

int wm8960_read(unsigned char reg_addr)
{
    unsigned int reg_num = sizeof(wm_8960_reg_defaults) / sizeof(struct reg_default);
    for (int i = 0; i < reg_num; ++i)
    {
      if (wm_8960_reg_defaults[i].reg == reg_addr)
      {
        int reg_val = wm_8960_reg_defaults[i].def;
        return reg_val;
      }
    }

    printk("wrong reg addr!!!\n");
    return -1;
}

static void wm8960_update_regTable(unsigned int reg_addr, unsigned int value)
{
    unsigned int reg_num = sizeof(wm_8960_reg_defaults) / sizeof(struct reg_default);
    for (int i = 0; i < reg_num; ++i)
    {
      if (wm_8960_reg_defaults[i].reg == reg_addr)
      {
        wm_8960_reg_defaults[i].def = value;
        return;
      }
    }

    printk("wrong reg addr!!!\n");
}

static int wm8960_register_update(unsigned int reg_addr, unsigned int mask, unsigned int value)
{
  unsigned int temp_val;
  unsigned int original_val = wm8960_read(reg_addr);
  temp_val = original_val & (~mask);
  temp_val = temp_val | (value & mask);
  int ret_val = wm8960_write(reg_addr, temp_val);
  if (ret_val < 0)
  {
    printk("%s: failed!!!",__FUNCTION__);
  } else {
    wm8960_update_regTable(reg_addr, temp_val);
    // update table;
  }
}

void wm8960_reg_dump(unsigned int reg_num)
{
    unsigned int i = 0;
    for(i = 0;i < reg_num;i++)
    {
        printk("reg%d =%x,",i,wm8960_read(i));
        if((i+1)%8==0)
        {
            printk("\n");
        }
    }
}

static void LINPUT3_MIC_BOOST_ADC_HP_initialization(unsigned int chip_num)
{
  unsigned int mask,value;

  value = VMIDSEL(VMID_FOR_LOWPOWER_STANDBY) |
          POWER_VREF(1) |
          POWER_AINL(1) |
          POWER_RINR(1) |
          POWER_ADCL(1) |
          POWER_ADCR(1) |
          POWER_MICB(1);
  mask = MASK_VMIDSEL | MASK_VREF | MASK_AINL | MASK_RINR | MASK_ADCL | MASK_ADCR | MASK_MICB;
  wm8960_register_update(WM8960_POWER1, mask, value);
  msleep(10);
  /*clock*/
  wm8960_write(WM8960_CLOCK1, 0x0);// SYSCLK derived from MCLK(12.288M) ADC/DAC Sample rate:48K
  wm8960_write(WM8960_CLOCK2, 0x1c4);//BCLK RATE12.288M MAXIMUM WORD LENGTH:32
  /*digital audio interface*/
  value = SWITCH_MS(digital_master_mode) |
          SWITCH_WL(Audio_Data_16bits) |
          SWITCH_FORMAT(Audio_Data_Format_I2S);
  mask = MASK_MS | MASK_WL | MASK_FORMAT;
  wm8960_register_update(WM8960_IFACE1, mask, value);
  mask = LOUT1_MASK | ROUT1_MASK;
  value = POWER_LOUT1(1) | 
          POWER_ROUT1(1);
  wm8960_register_update(WM8960_POWER2, mask, value);  
  msleep(10);
  mask = LMIC_MASK | RMIC_MASK | LOMIX_MASK | ROMIX_MASK;
  value = POWER_LMIC(1) |
          POWER_RMIC(1) |
          POWER_LOMIX(1) |
          POWER_ROMIX(1);
  wm8960_register_update(WM8960_POWER3, mask, value);

  /*input signal path*/
  // mic
  value = SWITCH_INMUTE(0)|INVOL_VALUE(INPUT_PGA_VOL_DB_21)|SWITCH_IPVU(1)|SWITCH_LIZC(1);
  mask = MASK_INMUTE|INVOL_MASK|MASK_IPVU|MASK_LIZC;
  wm8960_register_update(WM8960_LINVOL, mask, value);
  wm8960_register_update(WM8960_RINVOL, mask, value);

  value = SWITCH_MIC2B(1)|MICBOOST_GAIN(Input_PGA_Boost_Gain_0dB)|SWITCH_MN1(1);
  mask = MASK_MIC2B|MICBOOST_MASK|MASK_MN1;
  wm8960_register_update(WM8960_LINPATH, mask, value);
  wm8960_register_update(WM8960_RINPATH, mask, value);

  //[LINE INPUT 3]
  mask = IN3BOOST_MASK;
  value = IN3BOOST_GAIN(LINE_input_Boost_gain_DB_0);
  wm8960_register_update(WM8960_INBMIX1, mask, value);
  wm8960_register_update(WM8960_INBMIX2, mask, value);

  /*ADC Digital Volume Control*/
  mask = MASK_ADCVU | MASK_ADCVOL;
  value = SWITCH_ADCVU(1) | 
          ADCVOL(0);
  wm8960_register_update(WM8960_LADC, mask, value);
  wm8960_register_update(WM8960_RADC, mask, value);

  /* output path */
  //Output Mixer
  mask = B2O_MASK | B2OVOL_MASK;
  value = B2O_ENABLE(1) | 
          B2OVOL(B2O_VOL_DB_n12);
  wm8960_register_update(WM8960_BYPASS1, mask, value);
  wm8960_register_update(WM8960_BYPASS2, mask, value);
  // value = LD2LO;
  // wm8960_write(WM8960_LOUTMIX, value);
  // value = RD2LO;
  // wm8960_write(WM8960_ROUTMIX, value);
  
  //Headphone Volume
  mask = MASK_OUT1VU | MASK_O1ZC | MASK_OUT1VOL;
  value = SWITCH_OUT1VU(1) | 
          SWITCH_O1ZC(1) | 
          SWITCH_OUT1VOL(0);
  wm8960_register_update(WM8960_LOUT1, mask, value);
  wm8960_register_update(WM8960_ROUT1, mask, value);

  mask = MASK_ADCHPD;
  value = SWITCH_ADCHPD(0);
  wm8960_register_update(WM8960_DACCTL1, mask, value);
  mask = MASK_ALRCGPIO;
  value = SWITCH_ALRCGPIO(ADCLRC_GPIO1_Pin_GPIO);
  wm8960_register_update(WM8960_IFACE2, mask, value);

  // wm8960_write(WM8960_APOP1, 0x8);
  mask = MASK_TOEN;
  value = SWITCH_TOEN(1);
  wm8960_register_update(WM8960_ADDCTL1, mask, value);

  mask = MASK_HPSWEN | MASK_HPSWPOL | MASK_LRCM;
  value = SWITCH_HPSWEN(1) | 
          SWITCH_HPSWPOL(1) | 
          SWITCH_LRCM(1);
  wm8960_register_update(WM8960_ADDCTL2, mask, value);
  mask = HPSEL_MASK | GPIOSEL_MASK | MBSEL_MASK;  
  value = HPSEL(HP_SWITCH_SEL_GPIO1) |
          GPIOSEL(GPIO_Function_Jack_detect_input) |
          MBSEL(Mic_Bias_Voltage_0dot65_AVDD);  
  wm8960_register_update(WM8960_ADDCTL4, mask, value);
}

static void LINPUT3_MIC_BOOST_HP_initialization(unsigned int chip_num)
{
  // wm8960_write(WM8960_POWER1, 0xcc );//ENABLE ADC disaable MICBIAS
  wm8960_write(WM8960_POWER1, 0xfe );//ENABLE Analogue Input PGA and Boost MICBIAS
  msleep(250);
  //wm8960_write(WM8960_PLL1, 0x37);
  // wm8960_write(WM8960_PLL2, 0x86);
  // wm8960_write(WM8960_PLL3, 0xc2);
  // wm8960_write(WM8960_PLL4, 0x26);
  /*clock*/
  wm8960_write(WM8960_CLOCK1, 0x0);// SYSCLK derived from MCLK(12.288M) ADC/DAC Sample rate:48K
  wm8960_write(WM8960_CLOCK2, 0x1c0);//BCLK RATE12.288M MAXIMUM WORD LENGTH:32
  /*digital audio interface*/
  wm8960_write(WM8960_IFACE1, 0x42);//master mode[6]  I2S Format[0:1]
  // wm8960_write(WM8960_POWER2, 0x1E0);//ENANBLE DAC/OUT1 DISABLE PLLL SPEAKER OUT
  wm8960_write(WM8960_POWER2, 0x1F8);//ENANBLE DAC/OUT1 SPEAKER DISABLE PLLL  OUT
  msleep(250);
  wm8960_write(WM8960_POWER3, 0x3c);//Output Mixer/MIC Enable
  // wm8960_write(WM8960_POWER3, 0x3c);//Output Mixer Enable Input PGA Enable
  /*Input PGA Volume Control*/
  // wm8960_write(WM8960_LINVOL, 0x117);
  // wm8960_write(WM8960_RINVOL, 0x117);
  /*Headphone Volume*/
  // wm8960_write(WM8960_LOUT1, 0x17f);//+6db
  // wm8960_write(WM8960_ROUT1, 0x17f);//+6db
  wm8960_write(WM8960_LOUT1, 0x179);//+0db
  wm8960_write(WM8960_ROUT1, 0x179);//+0db

  wm8960_write(WM8960_DACCTL1, 0x0);
  wm8960_write(WM8960_IFACE2, 0x41);
  /*DAC Digital Volume Control */
  wm8960_write(WM8960_LDAC, 0xff);//0db
  wm8960_write(WM8960_RDAC, 0xff);//0db
  /*ADC Digital Volume Control*/
  wm8960_write(WM8960_LADC, 0xc3);//0db
  wm8960_write(WM8960_RADC, 0xc3);//0db
  wm8960_write(WM8960_APOP1, 0x8);
  //input signal path mic
  wm8960_write(WM8960_LINPATH, 0x108);//MICBOOST 0db
  wm8960_write(WM8960_RINPATH, 0x108);//MICBOOST 0db
  wm8960_write(WM8960_LINVOL, 0x13F);// PGA Volume 0db
  wm8960_write(WM8960_RINVOL, 0x13F);// PGA Volume 0db
  // wm8960_write(WM8960_LINVOL, 0x117);// PGA Volume 0db
  // wm8960_write(WM8960_RINVOL, 0x117);// PGA Volume 0db
  wm8960_write(WM8960_ADDCTL1, 0x1C1);//Slow clock enabled

  /*Output Mixer*/
  wm8960_write(WM8960_LOUTMIX, 0x00);//DISABLE DAC AND LINPUT3
  wm8960_write(WM8960_ROUTMIX, 0x00);//DISABLE DAC AND LINPUT3
  wm8960_write(WM8960_BYPASS1, 0x80);// Input Boost Mixer 2 output mixer
  wm8960_write(WM8960_BYPASS2, 0x80);// Input Boost Mixer 2 output mixer

  /*Speaker Volume:mute*/ 
  // wm8960_write(WM8960_LOUT2, 0x179);
  // wm8960_write(WM8960_ROUT2, 0x179);
  wm8960_write(WM8960_INBMIX1, 0x70);//LINPUT3 Boost Mixer Gain +6DB
  wm8960_write(WM8960_INBMIX2, 0x70);//RINPUT3 Boost Mixer Gain +6DB

  /*CLASS D speaker*/
  // wm8960_write(WM8960_CLASSD1, 0xf7);
  // wm8960_write(WM8960_CLASSD3, 0x9b);
  wm8960_write(WM8960_ADDCTL2, 0x60);//HPDETECT low = headphone
  wm8960_write(WM8960_ADDCTL4, 0x02);//ADCLRC/GPIO1 used for jack detect input
  // wm8960_write(WM8960_ADDCTL3, 0x40);//1 = 20kΩ VMID to output
}

static void LINPUT3_BYPASS_initialization(unsigned int chip_num)
{
  wm8960_write(WM8960_POWER1, 0xc0);//OK
  wm8960_write(WM8960_POWER2, 0x60);//enable LOUT1/ROUT1
  msleep(250);
  wm8960_write(WM8960_LINPATH, 0x0);
  wm8960_write(WM8960_RINPATH, 0x0);
  wm8960_write(WM8960_POWER3, 0xc);
  wm8960_write(WM8960_LOUTMIX, 0x80);//0DB
  wm8960_write(WM8960_ROUTMIX, 0x80);//0DB
  // wm8960_write(0x31, 0xf7);
  // wm8960_write(0x33, 0x9b);
  // wm8960_write(0x28, 0x179);
  // wm8960_write(0x29, 0x179);
  wm8960_write(WM8960_LOUT1, 0x179);//LOUT1 Volume +0db
  wm8960_write(WM8960_ROUT1, 0x179);//ROUT1 Volume +0db
}

void soft_reset(unsigned int chip_num)
{
    wm8960_write(WM8960_RESET,       0x00);
    msleep(250);
    // DAC_SPK_MIC_initialization(chip_num);
    // LINPUT3_BYPASS_initialization(chip_num);
    LINPUT3_MIC_BOOST_ADC_HP_initialization(chip_num);
    // LINPUT3_MIC_BOOST_HP_initialization(chip_num);
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
            in2_adc_ctrl_sample.b8 = wm8960_read(17);      
            in2_adc_ctrl_sample.bit.in2l_adc_input_level_sample = audio_ctrl->input_level;
            wm8960_write(17,in2_adc_ctrl_sample.b8);
            break;
        case IN2LR_2_RIGTH_ADC_CTRL:
            in2_adc_ctrl_sample.b8 = wm8960_read(18);      
            in2_adc_ctrl_sample.bit.in2r_adc_input_level_sample = audio_ctrl->input_level;
            wm8960_write(18,in2_adc_ctrl_sample.b8);
             
            break;
        case IN1L_2_LEFT_ADC_CTRL:
            in1_adc_ctrl.b8 = wm8960_read(19);      
            in1_adc_ctrl.bit.in1_adc_input_level = audio_ctrl->input_level;
            in1_adc_ctrl.bit.adc_ch_power_ctrl = audio_ctrl->if_powerup;
            wm8960_write(19,in1_adc_ctrl.b8);
            break;
        case IN1R_2_RIGHT_ADC_CTRL:
            in1_adc_ctrl.b8 = wm8960_read(22);      
            in1_adc_ctrl.bit.in1_adc_input_level = audio_ctrl->input_level;
            in1_adc_ctrl.bit.adc_ch_power_ctrl = audio_ctrl->if_powerup;
            wm8960_write(22,in1_adc_ctrl.b8);
            break;
        case PGAL_2_HPLOUT_VOL_CTRL:
            adc_pga_dac_gain_ctrl.b8 = wm8960_read(46);
            adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
            adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
            wm8960_write(46,adc_pga_dac_gain_ctrl.b8);
            break;
        case DACL1_2_HPLOUT_VOL_CTRL:
            adc_pga_dac_gain_ctrl.b8 = wm8960_read(47);
            adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
            adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
            wm8960_write(47,adc_pga_dac_gain_ctrl.b8);
            break;
        case HPLOUT_OUTPUT_LEVEL_CTRL:
            line_hpcom_out_ctrl.b8 =  wm8960_read(51); 
            line_hpcom_out_ctrl.bit.if_mute = audio_ctrl->if_mute_route;
            line_hpcom_out_ctrl.bit.output_level = audio_ctrl->input_level;
			line_hpcom_out_ctrl.bit.power_status = audio_ctrl->if_powerup;
            wm8960_write(51,line_hpcom_out_ctrl.b8);
            break; 
        case PGAL_2_HPLCOM_VOL_CTRL:
            adc_pga_dac_gain_ctrl.b8 = wm8960_read(53);
            adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
            adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
            wm8960_write(53,adc_pga_dac_gain_ctrl.b8);
            break;
        case DACL1_2_HPLCOM_VOL_CTRL:
            adc_pga_dac_gain_ctrl.b8 = wm8960_read(54);
            adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
            adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
            wm8960_write(54,adc_pga_dac_gain_ctrl.b8);
            break;
        case HPLCOM_OUTPUT_LEVEL_CTRL:
           line_hpcom_out_ctrl.b8 =  wm8960_read(58); 
           line_hpcom_out_ctrl.bit.if_mute = audio_ctrl->if_mute_route;
           line_hpcom_out_ctrl.bit.output_level =  audio_ctrl->input_level;
           wm8960_write(58,line_hpcom_out_ctrl.b8);
          break;
        case PGAR_2_HPROUT_VOL_CTRL:
            adc_pga_dac_gain_ctrl.b8 = wm8960_read(63);
            adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
            adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
            wm8960_write(63,adc_pga_dac_gain_ctrl.b8);
            break;
        case DACR1_2_HPROUT_VOL_CTRL: 
            adc_pga_dac_gain_ctrl.b8 = wm8960_read(64);
            adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
            adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
            wm8960_write(64,adc_pga_dac_gain_ctrl.b8);
            break;
        case HPROUT_OUTPUT_LEVEL_CTRL:
           line_hpcom_out_ctrl.b8 =  wm8960_read(65); 
           line_hpcom_out_ctrl.bit.if_mute = audio_ctrl->if_mute_route;
           line_hpcom_out_ctrl.bit.output_level = audio_ctrl->input_level;
		   line_hpcom_out_ctrl.bit.power_status = audio_ctrl->if_powerup;
           wm8960_write(65,line_hpcom_out_ctrl.b8);
           break;
        case PGAR_2_HPRCOM_VOL_CTRL: 
              adc_pga_dac_gain_ctrl.b8 = wm8960_read(70);
              adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
              adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
              wm8960_write(70,adc_pga_dac_gain_ctrl.b8);
              break;
        case DACR1_2_HPRCOM_VOL_CTRL:
             adc_pga_dac_gain_ctrl.b8 = wm8960_read(71);
              adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route; 
              adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
              wm8960_write(71,adc_pga_dac_gain_ctrl.b8);
                break;
        case HPRCOM_OUTPUT_LEVEL_CTRL:
              line_hpcom_out_ctrl.b8 =  wm8960_read(72); 
               line_hpcom_out_ctrl.bit.if_mute = audio_ctrl->if_mute_route;
               line_hpcom_out_ctrl.bit.output_level =  audio_ctrl->input_level;
               wm8960_write(72,line_hpcom_out_ctrl.b8);
              break;
        case PGAL_2_LEFT_LOP_VOL_CTRL:
              adc_pga_dac_gain_ctrl.b8 = wm8960_read(81);
              adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
              adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
              wm8960_write(81,adc_pga_dac_gain_ctrl.b8);
                break;
        case DACL1_2_LEFT_LOP_VOL_CTRL:
             adc_pga_dac_gain_ctrl.b8 = wm8960_read(82);
              adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route; 
              adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
              wm8960_write(82,adc_pga_dac_gain_ctrl.b8);
                break;
        case LEFT_LOP_OUTPUT_LEVEL_CTRL:
              line_hpcom_out_ctrl.b8 =  wm8960_read(86); 
               line_hpcom_out_ctrl.bit.if_mute = audio_ctrl->if_mute_route;
               line_hpcom_out_ctrl.bit.output_level =  audio_ctrl->input_level;
               line_hpcom_out_ctrl.bit.power_status =  audio_ctrl->if_powerup;
               wm8960_write(86,line_hpcom_out_ctrl.b8);
              break;
        case PGAR_2_RIGHT_LOP_VOL_CTRL:
              adc_pga_dac_gain_ctrl.b8 = wm8960_read(91);
              adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
              adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
              wm8960_write(91,adc_pga_dac_gain_ctrl.b8);
                break;
        case DACR1_2_RIGHT_LOP_VOL_CTRL:
             adc_pga_dac_gain_ctrl.b8 = wm8960_read(92);
              adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route; 
              adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
              wm8960_write(92,adc_pga_dac_gain_ctrl.b8);
                break;
        case RIGHT_LOP_OUTPUT_LEVEL_CTRL:
              line_hpcom_out_ctrl.b8 =  wm8960_read(93); 
               line_hpcom_out_ctrl.bit.if_mute = audio_ctrl->if_mute_route;
               line_hpcom_out_ctrl.bit.output_level =  audio_ctrl->input_level;
               line_hpcom_out_ctrl.bit.power_status =  audio_ctrl->if_powerup;
               wm8960_write(93,line_hpcom_out_ctrl.b8);
              break;
        case SET_ADC_SAMPLE:
                in2_adc_ctrl_sample.b8 = wm8960_read(2);      
                in2_adc_ctrl_sample.bit.in2l_adc_input_level_sample = audio_ctrl->sample;
                wm8960_write(2,in2_adc_ctrl_sample.b8);
                break;
        case SET_DAC_SAMPLE:
                in2_adc_ctrl_sample.b8 = wm8960_read(2);      
                in2_adc_ctrl_sample.bit.in2r_adc_input_level_sample = audio_ctrl->sample;
                wm8960_write(2,in2_adc_ctrl_sample.b8);
                //printk("set SET_DAC_SAMPLE,audio_ctrl->sample=%x\n",audio_ctrl->sample);
                break;
        case SET_DATA_LENGTH:
                serial_int_ctrl.b8 = wm8960_read(9);;                
                serial_int_ctrl.bit.data_length = audio_ctrl->data_length;
                //wm8960_write(9,serial_int_ctrl.b8);
                break;
        case SET_TRANSFER_MODE:
                serial_int_ctrl.b8 = wm8960_read(9);    
                serial_int_ctrl.bit.transfer_mode = audio_ctrl->trans_mode;
                wm8960_write(9,serial_int_ctrl.b8);
                break;              
        case SET_CTRL_MODE:
                //wm8960_write(0x1,0x80);
                //udelay(50);  
                ctrl_mode.b8 = wm8960_read(8);
                ctrl_mode.bit.bit_clock_dic_ctrl =  audio_ctrl->ctrl_mode;
                ctrl_mode.bit.work_clock_dic_ctrl =  audio_ctrl->ctrl_mode;
                ctrl_mode.bit.bit_work_dri_ctrl =  audio_ctrl->ctrl_mode;
                wm8960_write(8,ctrl_mode.b8);
#if 0
                /* ÉèÖÃÊ±ÖÓ */
                if (1 == audio_ctrl->ctrl_mode 
                    || (AC31_SET_48K_SAMPLERATE != audio_ctrl->sample && AC31_SET_44_1K_SAMPLERATE != audio_ctrl->sample))
                {
                    /* aic31×÷Ö÷Ä£Ê½»òÕß²ÉÑùÂÊ²»Îª44.1K/48KHZµÄÇé¿öÏÂ£¬Ê¹ÓÃÍâ²¿µÄ12.288MHZµÄ¾§Õñ×÷ÎªMCLKÊäÈë²¢²úÉúÄÚ²¿¹¤×÷Ö÷Ê±ÖÓ */
                    if ((1 == audio_ctrl->if_44100hz_series))
                    {
                        /*¡¡Èç¹ûÎª44.1KHZÏµÁÐµÄ²ÉÑùÑù */
                        wm8960_write(3,0x81);    /* P=1 */ 
                        wm8960_write(4,0x1c);    /* J=7 */
                        wm8960_write(5,0x36);    /* reg 5 and 6 set D=3500*/
                        wm8960_write(6,0xb0);
                        codec_datapath_setup_ctrl.b8 = wm8960_read(7);
                        codec_datapath_setup_ctrl.b8 |= 0x80;   /* FSref = 44.1 kHz */
                        wm8960_write(7,codec_datapath_setup_ctrl.b8);
                        wm8960_write(11,0x1);    /* R=1 */
                        wm8960_write(101,0x0);
                        wm8960_write(102,0xc2);
                    }
                    else
                    {
                        /*¡¡Èç¹ûÎª·Ç44.1KHZÏµÁÐµÄ²ÉÑùÑù */
                        wm8960_write(3,0x81);    /* P=1 */ 
                        wm8960_write(4,0x20);    /* J=8 */
                        wm8960_write(5,0x0);     /* reg 5 and 6 set D=0000*/
                        wm8960_write(6,0x0);
                        codec_datapath_setup_ctrl.b8 = wm8960_read(7);
                        codec_datapath_setup_ctrl.b8 &= 0x7f;   /* FSref = 48 kHz */
                        wm8960_write(7,codec_datapath_setup_ctrl.b8);
                        wm8960_write(11,0x1);    /* R=1 */
                        wm8960_write(101,0x0);
                        wm8960_write(102,0xc2);
                    }
                }
                else
                {
                    /* aic31×ö´ÓÄ£Ê½ÇÒ²ÉÑùÂÊÎª44.1K/48KHZµÄÇé¿öÏÂ£¬ÓÉBCLK²úÉúÄÚ²¿¹¤×÷Ö÷Ê±ÖÓ */
                    wm8960_write(102,0x22);  /* uses PLLCLK and BCLK */
                    codec_datapath_setup_ctrl.b8 = wm8960_read(7);
                    if ((1 == audio_ctrl->if_44100hz_series))
                    {
                        codec_datapath_setup_ctrl.b8 |= 0x80;   /* FSref = 44.1 kHz */
                    }
                    else
                    {
                        codec_datapath_setup_ctrl.b8 &= 0x7f;   /* FSref = 48 kHz */
                    }
                    wm8960_write(7,codec_datapath_setup_ctrl.b8);

                    wm8960_write(3,0x81);    /* P=1 */ 
                    wm8960_write(4,32<<2);   /* set PLL J to 32 */
                    wm8960_write(5,0x0);     /* reg 5 and 6 set D=0000*/
                    wm8960_write(6,0x0);
                    wm8960_write(101,0x0);   /* CODEC_CLKIN uses PLLDIV_OUT */
                    wm8960_write(11,0x2);    /* R = 2 */
                }
#else
 				/* ÉèÖÃÊ±ÖÓ */
               /* aic31,ÓÉaiaoÌá¹©mclk */             
          		switch(audio_ctrl->sampleRate)
            	{
            		case 8000:
            		case 16000:
            		case 32000:
            			{
            			    /*¡¡Èç¹ûÎª32KHZÏµÁÐµÄ²ÉÑùÑù */
	                        wm8960_write(3,0x81);    /* P=1 */ 
	                        wm8960_write(4,0x30);    /* J=12 */
	                        wm8960_write(5,0x0);     /* reg 5 and 6 set D=0000*/
	                        wm8960_write(6,0x0);
	                        codec_datapath_setup_ctrl.b8 = wm8960_read(7);
	                        codec_datapath_setup_ctrl.b8 &= 0x7f;   /* FSref = 48 kHz */
	                        wm8960_write(7,codec_datapath_setup_ctrl.b8);
	                        wm8960_write(11,0x1);    /* R=1 */
	                        wm8960_write(101,0x0);
	                        wm8960_write(102,0xc2);
            			}
            			break;
            		case 12000:
            		case 24000:
            		case 48000:                		
            		    {
            		        /*¡¡Èç¹ûÎª48KHZÏµÁÐµÄ²ÉÑùÑù */
	                        wm8960_write(3,0x81);    /* P=1 */ 
	                        wm8960_write(4,0x20);    /* J=8 */
	                        wm8960_write(5,0x0);     /* reg 5 and 6 set D=0000*/
	                        wm8960_write(6,0x0);
	                        codec_datapath_setup_ctrl.b8 = wm8960_read(7);
	                        codec_datapath_setup_ctrl.b8 &= 0x7f;   /* FSref = 48 kHz */
	                        wm8960_write(7,codec_datapath_setup_ctrl.b8);
	                        wm8960_write(11,0x1);    /* R=1 */
	                        wm8960_write(101,0x0);
	                        wm8960_write(102,0xc2);
                        }
            			break;
            		case 11025:
            		case 22050:
            		case 44100:
            		    {
                		    /*¡¡Èç¹ûÎª44.1KHZÏµÁÐµÄ²ÉÑùÑù */
	                        wm8960_write(3,0x81);    /* P=1 */ 
	                        wm8960_write(4,0x20);    /* J=7 */
	                        wm8960_write(5,0x00);    /* reg 5 and 6 set D=0000*/
	                        wm8960_write(6,0x00);
	                        codec_datapath_setup_ctrl.b8 = wm8960_read(7);
	                        codec_datapath_setup_ctrl.b8 |= 0x80;   /* FSref = 44.1 kHz */
	                        wm8960_write(7,codec_datapath_setup_ctrl.b8);
	                        wm8960_write(11,0x1);    /* R=1 */
	                        wm8960_write(101,0x0);
	                        wm8960_write(102,0xc2);
            		    }
            			break;               			
                
                default:
                	printk("aic31 unsupport sampleRate %d\n", audio_ctrl->sampleRate);
                	return -1;
            	}	
               
#endif
                break;
        case LEFT_DAC_VOL_CTRL:
                adc_pga_dac_gain_ctrl.b8 = wm8960_read(43);
                adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route; 
                adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
                wm8960_write(43,adc_pga_dac_gain_ctrl.b8);
                break;
        case RIGHT_DAC_VOL_CTRL:
                adc_pga_dac_gain_ctrl.b8 = wm8960_read(44);
                adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route; 
                adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
                wm8960_write(44,adc_pga_dac_gain_ctrl.b8);
                break;
        case LEFT_DAC_POWER_SETUP:
                codec_datapath_setup_ctrl.b8 = wm8960_read(7);
                codec_datapath_setup_ctrl.bit.left_dac_datapath_ctrl = audio_ctrl->if_powerup;
                wm8960_write(7,codec_datapath_setup_ctrl.b8);
                dac_power_ctrl.b8 =  wm8960_read(37);
                dac_power_ctrl.bit.left_dac_power_ctrl =  audio_ctrl->if_powerup;
                wm8960_write(37,dac_power_ctrl.b8);
                 break;
        case RIGHT_DAC_POWER_SETUP:
                codec_datapath_setup_ctrl.b8 = wm8960_read(7);
                codec_datapath_setup_ctrl.bit.right_dac_datapath_ctrl = audio_ctrl->if_powerup;
                wm8960_write(7,codec_datapath_setup_ctrl.b8);
                dac_power_ctrl.b8 =  wm8960_read(37);
                dac_power_ctrl.bit.right_dac_power_ctrl =  audio_ctrl->if_powerup;
                wm8960_write(37,dac_power_ctrl.b8);
                 break;
        case DAC_OUT_SWITCH_CTRL:  
                dac_output_swit_ctrl.b8 = wm8960_read(41);
                dac_output_swit_ctrl.bit.left_dac_swi_ctrl =  audio_ctrl->dac_path;
                dac_output_swit_ctrl.bit.right_dac_swi_ctrl = audio_ctrl->dac_path;
                wm8960_write(41,dac_output_swit_ctrl.b8);
                 break;
        case LEFT_ADC_PGA_CTRL:
                adc_pga_dac_gain_ctrl.b8 = wm8960_read(15);
                adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
                adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
                wm8960_write(15,adc_pga_dac_gain_ctrl.b8);
                break;
        case RIGHT_ADC_PGA_CTRL:
                adc_pga_dac_gain_ctrl.b8 = wm8960_read(16);
                adc_pga_dac_gain_ctrl.bit.if_mute_route = audio_ctrl->if_mute_route;
                adc_pga_dac_gain_ctrl.bit.input_vol_level_ctrl = audio_ctrl->input_level;
                wm8960_write(16,adc_pga_dac_gain_ctrl.b8);
                break;
        case SET_SERIAL_DATA_OFFSET:
                serial_data_offset_ctrl.b8 = wm8960_read(10);
                serial_data_offset_ctrl.bit.serial_data_offset = audio_ctrl->data_offset;
                wm8960_write(10,serial_data_offset_ctrl.b8);
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
        wm8960_write(51, 0x04);
        wm8960_write(65, 0x04);
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
    unsigned int reg_num = sizeof(wm_8960_reg_defaults) / sizeof(struct reg_default);
    for (int i = 0; i < reg_num; ++i)
    {   
        reg_addr = wm_8960_reg_defaults[i].reg;
        value    = wm_8960_reg_defaults[i].def;
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

