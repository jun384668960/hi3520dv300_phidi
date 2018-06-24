#include <linux/module.h>	//所有模块都需要的头文件
#include <linux/kernel.h>
#include <linux/fs.h>	//文件系统有关的，结构体file_operations也在fs头文件定义
#include <linux/init.h>	//init和exit相关宏
#include <linux/delay.h>
#include <linux/irq.h>
#include <asm/uaccess.h>	//linux中的用户态内存交互函数，copy_from_user(),copy_to_user()等
#include <asm/irq.h>	//linux中断定义
#include <asm/io.h>
#include <linux/sched.h>	//声明printk()这个内核态的函数
#include <linux/interrupt.h>	//包含与中断相关的大部分宏及结构体的定义，request_irq()等
#include <linux/device.h>	//
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/gfp.h>
#include "led_drv.h"

static struct class *leddrv_class;
static struct device *leddrv_dev;

static DECLARE_WAIT_QUEUE_HEAD(led_waitq);

struct gpio_desc{                  //定义结构体
	unsigned int mux_ofset_addr;	//复用寄存器偏移地址
	unsigned int mux_val;			//复用寄存的设置值
	unsigned int gpio_base_addr;	//gpio寄存器基址
	unsigned int irq;				//中断号
	unsigned int key_val;			//值
	unsigned char bit;				//对gpio寄存进行位操作时用，范围bit7~bit0
	unsigned int number;			//序号
	char *name;						//描述（即按键名）
};

struct led_desc{
	unsigned int  key_val;
	unsigned char status;
};

struct gpio_desc leds_desc[] = {     //定义一个结构体数组
	{GPIO10_4_MUX_CTRL_REG_OFFSET, 0x00, GPIO10_BASE, IRQ_GPIO10_4, 0x01, 4, 0,  "LED_0" },
};

/*********************************************************************************************************
*功能：设置引脚复用
*参数：无
*返回值：无
**********************************************************************************************************/
static void gpio_mux_ctrl_set(void)
{
	unsigned int i;

	for(i=0; i<sizeof(leds_desc)/sizeof(leds_desc[0]); i++){
		GPIO_WRITE_REG(MUX_CTRL_REG_BASE, leds_desc[i].mux_ofset_addr, leds_desc[i].mux_val);
	}
}

/*********************************************************************************************************
*功能：GPIO相关寄存器设置
*参数：base_addr-->基地址, offset_addr-->偏移地址, bit-->位, val-->值
*返回值：无
**********************************************************************************************************/
static void gpio_reg_set(unsigned int base_addr, unsigned int offset_addr, unsigned char bit, unsigned char val)
{
	unsigned int value;

	if(val > 1){
		printk("<val:%d> input error!\n", val);
	}

	if(bit > 7){
		printk("<bit:%d> input error!\n", bit);
	}
	value = ((GPIO_READ_REG(base_addr,  offset_addr) & (~(1<<bit))) | (val<<bit));
	GPIO_WRITE_REG(base_addr,offset_addr,value);
}

// /*********************************************************************************************************
// *功能：设置引脚上的电平
// *参数：base_addr-->基地址, offset_addr-->偏移地址, bit-->位, val-->值 0:低电平 1：高电平
// *返回值：返回引脚电平
// **********************************************************************************************************/
static void gpio_set_value(unsigned int base_addr, unsigned int offset_addr, unsigned char bit, unsigned char val)
{
	gpio_reg_set(base_addr,offset_addr,bit,val);
}

/*********************************************************************************************************
*功能：获取引脚上的电平
*参数：base_addr-->基地址, offset_addr-->偏移地址, bit-->位
*返回值：返回引脚电平
**********************************************************************************************************/
static unsigned char gpio_get_value(unsigned int base_addr, unsigned int offset_addr, unsigned char bit)
{
	unsigned char val;
	
	val = GPIO_READ_REG(base_addr,offset_addr) & (1<<bit);
	return val;
}

/*********************************************************************************************************
*功能：GPIO方向设置
*参数：base_addr-->基地址, offset_addr-->偏移地址, bit-->位, val-->值 0：输入 1：输出
*返回值：无
**********************************************************************************************************/
static void gpio_dir_set(unsigned int base_addr, unsigned int offset_addr, unsigned char bit, unsigned char val)
{
	gpio_reg_set(base_addr,offset_addr,bit,val);
}

/*********************************************************************************************************
*功能：GPIO相关寄存器组设置
*参数：无
*返回值：无
**********************************************************************************************************/
static void gpio_reg_group_set(void)
{
	int i;
	for(i=0; i<sizeof(leds_desc)/sizeof(leds_desc[0]); i++)
	{
		gpio_dir_set(leds_desc[i].gpio_base_addr, GPIO_DIR, leds_desc[i].bit, 1);	//0:输入 1：输出
	}
}

/*********************************************************************************************************
*功能：模块打开函数
*参数：
*返回值：无
**********************************************************************************************************/
static int led_drv_open(struct inode *inode, struct file *file)
{
	gpio_reg_group_set();

	return 0;
}

/*********************************************************************************************************
*功能：模块写入函数
*参数：
*返回值：无
**********************************************************************************************************/
ssize_t led_drv_write(struct file *file, const char __user *buf, size_t size, loff_t *ppos)
{
	int i;
	struct led_desc desc;
	
	if (size != sizeof(desc) || buf == NULL)
		return -EINVAL;

	if(copy_from_user(&desc, buf, size))  
    {  
        return -EFAULT;  
    }

	for(i=0; i<sizeof(leds_desc)/sizeof(leds_desc[0]); i++)
	{
		if(leds_desc[i].key_val == desc.key_val)
		{
			gpio_set_value(leds_desc[i].gpio_base_addr, GPIO_DATA, leds_desc[i].bit, desc.status);
		}
	}

	return 1;
}



/*********************************************************************************************************
*功能：模块关闭函数
*参数：
*返回值：无
**********************************************************************************************************/
int led_drv_close(struct inode *inode, struct file *file)     //出链，禁止中断
{
	return 0;
}



static struct file_operations led_drv_fops = {
    .owner   =  THIS_MODULE,    // 这是一个宏，推向编译模块时自动创建的__this_module变量
    .open    =  led_drv_open,     
	.write	 =	led_drv_write,
	.release =  led_drv_close,
};


int major;
/*********************************************************************************************************
*功能：模块加载函数
*参数：
*返回值：无
**********************************************************************************************************/
static int led_drv_init(void)
{

	gpio_mux_ctrl_set();	//引脚复用设置
	major = register_chrdev(0, "led_drv", &led_drv_fops);	//自动分配主设备号

	leddrv_class = class_create(THIS_MODULE, "led_drv");

	leddrv_dev = device_create(leddrv_class, NULL, MKDEV(major, 0), NULL, "led"); // 设备节点 /dev/led


	return 0;
}

/*********************************************************************************************************
*功能：模块卸载函数
*参数：
*返回值：无
**********************************************************************************************************/
static void led_drv_exit(void)
{
	unregister_chrdev(major, "led_drv");
	device_unregister(leddrv_dev);
	class_destroy(leddrv_class);
}


module_init(led_drv_init);
module_exit(led_drv_exit);
MODULE_DESCRIPTION("Led Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hisilicon");
