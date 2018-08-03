///*****************************************
//  Copyright (C) 2009-2014
//  ITE Tech. Inc. All Rights Reserved
//  Proprietary and Confidential
///*****************************************
//   @file   <Main.c>
//   @author Max.Kao@ite.com.tw
//   @date   2014/06/26
//   @fileversion: ITE_MHLRX_SAMPLE_V1.10
//******************************************/
//#include "config.h"
//#include "IO.h"
#include "Mhlrx.h"




static struct timer_list checker;

static int checker_handler(unsigned long data)
{
  //unsigned char mDataIn;
  //printk("IT6802_fsm\n");  
	IT6802_fsm();
	//i2c_read_byte(0x90, 0x75, 1, &mDataIn, 0);
	//printk("0x75 0x%x\n",mDataIn); 

	
	checker.expires = HZ / 50 + jiffies;
	add_timer(&checker);

	return 0;
}

int it6801_open(struct inode * inode, struct file * file)
{
    return 0;
}

int it6801_close(struct inode * inode, struct file * file)
{
    return 0;
}

static int it6801_vin_get_width(void)
{
	unsigned char mDataIn;
	i2c_read_byte(0x90, 0x9E, 1, &mDataIn, 0);
	int width = mDataIn;
	i2c_read_byte(0x90, 0x9F, 1, &mDataIn, 0);
	width = width | ((mDataIn & 0b00111111) << 8);
	
    return width;
}

static int it6801_vin_get_heigth(void)
{
	unsigned char mDataIn;
	i2c_read_byte(0x90, 0xA4, 1, &mDataIn, 0);
	int height = (mDataIn >> 4) << 8;
	i2c_read_byte(0x90, 0xA5, 1, &mDataIn, 0);
	height = height | mDataIn;

    return height;
}

typedef enum vin_resolution_id {
	VIN_RESOLUTION_1920x1080P  = 1,
	VIN_RESOLUTION_1280x1024P  = 5,
	VIN_RESOLUTION_1280x800P   = 6,
	VIN_RESOLUTION_1280x768P   = 7,
	VIN_RESOLUTION_1280x720P   = 2,
	VIN_RESOLUTION_720x576P    = 3,
	VIN_RESOLUTION_720x480P    = 4,
	VIN_RESOLUTION_640x480P    = 8,

	VIN_RESOLUTION_UNKOWN      = -1
} vin_resolution_id_e;


static bool it6801_is_power_5v_detected(void)
{
	unsigned char mDataIn;	
	i2c_read_byte(0x90, 0x0A, 1, &mDataIn, 0);
	// printk("reg 0x0A: 0x%x\n",mDataIn); 
	bool is_power_5V_detected = mDataIn & 0x1;

	return is_power_5V_detected;
}

static vin_resolution_id_e it6801_vin_get_resolution(void)
{
	int width  = it6801_vin_get_width();
	int height = it6801_vin_get_heigth();	
	// printk("Vin width[%d],height[%d]\n",width,height); 
	switch(width)
	{
		case 1920:
			return VIN_RESOLUTION_1920x1080P;
			break;
		case 1280:
			switch(height)
			{
				case 1024:
					return VIN_RESOLUTION_1280x1024P;
					break;
				case 800:
					return VIN_RESOLUTION_1280x800P;	
					break;
				case 768:
					return VIN_RESOLUTION_1280x768P;
					break;
				case 720:
					return VIN_RESOLUTION_1280x720P;	
					break;
				default:
					return VIN_RESOLUTION_UNKOWN;
					break;
			}
			break;
		case 720:
			switch(height)
			{
				case 576:
					return VIN_RESOLUTION_720x576P;
					break;
				case 480:
					return VIN_RESOLUTION_720x480P;	
					break;
				default:
					return VIN_RESOLUTION_UNKOWN;
					break;
			}
			break;
		case 640:
			return VIN_RESOLUTION_640x480P;
		default:
			printk("Vin: unknown resolution");
			return VIN_RESOLUTION_UNKOWN;
			break;
	}
}

typedef enum vin_frame_mode {
	frame_progressive_mode = 0,
	frame_interlaced_mode  = 1
} vin_frame_mode_e;

static vin_frame_mode_e it6801_vin_get_frame_mode(void)
{
	unsigned char mDataIn;
	//i2c_read_byte(0x90, 0x9A, 1, &mDataIn, 0);
	//printk("PCLK0x9A 0x%x\n",mDataIn); 
	//i2c_read_byte(0x90, 0x9D, 1, &mDataIn, 0);
	//printk("0x9D 0x%x\n",mDataIn); 
	i2c_read_byte(0x90, 0x99, 1, &mDataIn, 0);
	//printk("mDataIn 0x%x\n",mDataIn);  
	bool is_interlaced_mode = mDataIn & 0x02;
	return ((is_interlaced_mode) ? frame_interlaced_mode : frame_progressive_mode);

}

int it6801_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
		unsigned char mDataIn;

	if (false == it6801_is_power_5v_detected()) {
		return -1;
	}
		
	if(cmd == 0x10)
	{
		return it6801_vin_get_frame_mode();
	}
	
	if(cmd == 0x12)
	{
		return it6801_vin_get_resolution();
	}
	
	if(cmd == 0x15)
	{
		if(askVmode())
		{
					//printk("Input Video mode change interrupt\n");
					return 1;
		}				
		
	}
	
	if(cmd == 0x18)
	{
		i2c_read_byte(0x90, 0x0A, 1, &mDataIn, 0);
		printk("reg 0x0A: 0x%x\n",mDataIn); 
		if((mDataIn&0x1))  //5V detect
			return 1;
	}
	
	if(cmd == 0x20)
	{
		//i2c_read_byte(0x90, 0xAE, 1, &mDataIn, 0);
		//printk("reg 0xAE: 0x%x\n",mDataIn); 
		//i2c_read_byte(0x90, 0xAF, 1, &mDataIn, 0);
		//printk("reg 0xAF: 0x%x\n",mDataIn); 
		i2c_read_byte(0x90, 0x0A, 1, &mDataIn, 0);
		//printk("reg 0x0A: 0x%x\n",mDataIn); 
		if((mDataIn&0x80))  //Video Stable
			return 1;
	}
	
	if(cmd == 0x25)
	{
		//i2c_read_byte(0x90, 0xAE, 1, &mDataIn, 0);
		//printk("reg 0xAE: 0x%x\n",mDataIn); 
		//i2c_read_byte(0x90, 0xAF, 1, &mDataIn, 0);
		//printk("reg 0xAF: 0x%x\n",mDataIn); 
		i2c_read_byte(0x90, 0xAA, 1, &mDataIn, 0);
		//printk("reg 0x0A: 0x%x\n",mDataIn); 
		if((mDataIn!=0))  //aInput Stable
			return 1;
	}
	
	if(cmd == 0x30)
	{
		//i2c_read_byte(0x90, 0x9A, 1, &mDataIn, 0);
		//printk("PCLK0x9A 0x%x\n",mDataIn); 
		//i2c_read_byte(0x90, 0x2D, 1, &mDataIn, 0);
		//printk("reg 0x2D: 0x%x\n",mDataIn); 
		i2c_read_byte(0x90, 0x99, 1, &mDataIn, 0);
		//printk("0x99 0x%x\n",mDataIn); 
		
		//i2c_read_byte(0x90, 0x0C, 1, &mDataIn, 0);
		//printk("reg 0x0C: 0x%x\n",mDataIn); 
		
		
		if(mDataIn&0x08) //0x99 Indicate if video signal is stable .
		//if(mDataIn&0x40)	//0xC P0_RX_CLK stable
			return 1;
	}

    return 0;
}

static struct file_operations it6801_fops = 
{
    .owner      = THIS_MODULE,
    .open       = it6801_open,
    .release    = it6801_close,
    .unlocked_ioctl      = it6801_ioctl,
};

static struct miscdevice it6801_dev = 
{   
    .minor		= MISC_DYNAMIC_MINOR,
    .name		= "it6801",
    .fops  		= &it6801_fops,
};



static int __init it6801_init(void)
{
	//unsigned char ret;

    if (misc_register(&it6801_dev))
    {
        printk("ERROR: could not register it6801 devices\n");
		return -1;
    }
    
    hi_dev_init();
    it6802HPDCtrl(1,0);	// HDMI port , set HPD = 0

		delay1ms(1000);	//for power sequence
		
		//printk("IT6802_fsm_init\n");
		IT6802_fsm_init();//initialize registers


		init_timer(&checker);
		checker.function = (void *)checker_handler;

		checker.expires =  200 + jiffies;
		add_timer(&checker);

    return 0;
}

static void __exit it6801_exit(void)
{
	
    misc_deregister(&it6801_dev);
}

module_init(it6801_init);
module_exit(it6801_exit);

MODULE_LICENSE("GPL");

