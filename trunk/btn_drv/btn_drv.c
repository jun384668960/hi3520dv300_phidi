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
#include "btn_drv.h"

static struct class *btndrv_class;
static struct device *btndrv_dev;

static DECLARE_WAIT_QUEUE_HEAD(button_waitq);

// 中断事件标志, 中断服务程序将它置1，btn_drv_read将它清0 
static volatile int ev_press = 0;

static struct fasync_struct *button_async;  //定义一个结构

struct key_desc{                  //定义结构体
	unsigned int mux_ofset_addr;	//复用寄存器偏移地址
	unsigned int mux_val;			//复用寄存的设置值
	unsigned int gpio_base_addr;	//gpio寄存器基址
	unsigned int irq;				//中断号
	unsigned int key_val;			//按键值
	unsigned char bit;				//对gpio寄存进行位操作时用，范围bit7~bit0
	unsigned int number;			//按键序号
	char *name;						//按键描述（即按键名）
};

/* 键值: 按下时, 0x00, 0x01, 0x02, 0x03, 0x04 , 0x05, 0x06, 0x07, 0x08, 0x09*/
/* 键值: 松开时, 0x80, 0x81, 0x82, 0x83, 0x84 , 0x85, 0x86, 0x87, 0x88, 0x89*/ 
unsigned char *key_val = NULL;

struct key_desc keys_desc[] = {     //定义一个结构体数组
	{GPIO06_2_MUX_CTRL_REG_OFFSET, 0x01, GPIO06_BASE, IRQ_GPIO06_2, 0x01, 2, 0,  "KEY_0" },
};

/*********************************************************************************************************
*功能：设置引脚复用
*参数：无
*返回值：无
**********************************************************************************************************/
static void gpio_mux_ctrl_set(void)
{
	unsigned int i;

	for(i=0; i<sizeof(keys_desc)/sizeof(keys_desc[0]); i++){
		GPIO_WRITE_REG(MUX_CTRL_REG_BASE, keys_desc[i].mux_ofset_addr, keys_desc[i].mux_val);
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
		printk("<val:%d> input error!", val);
	}

	if(bit > 7){
		printk("<bit:%d> input error!", bit);
	}
	value = ((GPIO_READ_REG(base_addr,  offset_addr) & (~(1<<bit))) | (val<<bit));
	GPIO_WRITE_REG(base_addr,offset_addr,value);
}

// /*********************************************************************************************************
// *功能：设置引脚上的电平
// *参数：base_addr-->基地址, offset_addr-->偏移地址, bit-->位, val-->值 0:低电平 1：高电平
// *返回值：返回引脚电平
// **********************************************************************************************************/
//static void gpio_set_value(unsigned int base_addr, unsigned int offset_addr, unsigned char bit, unsigned char val)
//{
//	gpio_reg_set(base_addr,offset_addr,bit,val);
//}

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
*功能：GPIO管脚触发电平方式设置
*参数：base_addr-->基地址, offset_addr-->偏移地址, bit-->位, val-->值	0：边沿触发中断 1：电平触发中断
*返回值：无
**********************************************************************************************************/
static void gpio_is_set(unsigned int base_addr, unsigned int offset_addr, unsigned char bit, unsigned char val)
{
	gpio_reg_set(base_addr,offset_addr,bit,val);
}

/*********************************************************************************************************
*功能：GPIO双边沿沿触发设置
*参数：base_addr-->基地址, offset_addr-->偏移地址, bit-->位, val-->值 0：单边沿触发中断 1：双边沿触发中断
*返回值：无
**********************************************************************************************************/
static void gpio_ibe_set(unsigned int base_addr, unsigned int offset_addr, unsigned char bit, unsigned char val)
{
	gpio_reg_set(base_addr,offset_addr,bit,val);
}
#if 0
/*********************************************************************************************************
*功能：GPIO触发中断条件设置
*参数：base_addr-->基地址, offset_addr-->偏移地址, bit-->位, val-->值 0：下降沿(或低电平)触发中断 1：上升沿(或高电平)触发中断
*返回值：无
**********************************************************************************************************/
static void gpio_iev_set(unsigned int base_addr, unsigned int offset_addr, unsigned char bit, unsigned char val)
{
	gpio_reg_set(base_addr,offset_addr,bit,val);
}
#endif
/*********************************************************************************************************
*功能：GPIO中断屏蔽设置
*参数：base_addr-->基地址, offset_addr-->偏移地址, bit-->位, val-->值 0：屏蔽中断 1：不屏蔽中断
*返回值：无
**********************************************************************************************************/
static void gpio_ie_set(unsigned int base_addr, unsigned int offset_addr, unsigned char bit, unsigned char val)
{
	gpio_reg_set(base_addr,offset_addr,bit,val);
}

/*********************************************************************************************************
*功能：GPIO获取中断状态
*参数：base_addr-->基地址, offset_addr-->偏移地址, bit-->位
*返回值：0：中断无效 1：中断有效
**********************************************************************************************************/
static unsigned char gpio_mis_get_value(unsigned int base_addr, unsigned int offset_addr, unsigned char bit)
{
	unsigned char val;
	
	val = GPIO_READ_REG(base_addr,offset_addr) & (1<<bit);
	return val;
}

/*********************************************************************************************************
*功能：GPIO中断清除设置
*参数：base_addr-->基地址, offset_addr-->偏移地址, bit-->位, val-->值 0：不影响 1：清除中断
*返回值：无
**********************************************************************************************************/
static void gpio_ic_set(unsigned int base_addr, unsigned int offset_addr, unsigned char bit, unsigned char val)
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
	for(i=0; i<sizeof(keys_desc)/sizeof(keys_desc[0]); i++)
	{
		gpio_dir_set(keys_desc[i].gpio_base_addr, GPIO_DIR, keys_desc[i].bit, 0);	//0:输入 1：输出
		gpio_is_set( keys_desc[i].gpio_base_addr, GPIO_IS,	keys_desc[i].bit, 0);	//0：边沿触发中断 1：电平触发中断
		gpio_ibe_set(keys_desc[i].gpio_base_addr, GPIO_IBE, keys_desc[i].bit, 1);	//0：单边沿触发中断 1：双边沿触发中断
//		gpio_iev_set(keys_desc[i].gpio_base_addr, GPIO_IEV, keys_desc[i].bit, 0);	//0：下降沿(低电平)触发中断 1：上升沿(高电平)触发中断
		gpio_ie_set( keys_desc[i].gpio_base_addr, GPIO_IE,	keys_desc[i].bit, 1);	//0：屏蔽中断 1：不屏蔽中断
		gpio_ic_set( keys_desc[i].gpio_base_addr, GPIO_IC,	keys_desc[i].bit, 1);	//0：不影响 1：清除中断
	}
}

/*********************************************************************************************************
*功能：中断处理函数
*参数：irq-->中断号 dev_id-->设备ID
*返回值：无
**********************************************************************************************************/
static irqreturn_t buttons_irq(int irq, void *dev_id)         
{
	struct key_desc *keydesc = (struct key_desc *)dev_id;    //定义一个结构体指针使他的初值为ID
	unsigned int pinval;
	unsigned char flag;
	
	flag = gpio_mis_get_value(keydesc->gpio_base_addr, GPIO_MIS, keydesc->bit);	//获取中断状态
	if(!flag){	//非本模块中断
		return IRQ_RETVAL(IRQ_NONE);
	}
	pinval = gpio_get_value(keydesc->gpio_base_addr, GPIO_DATA, keydesc->bit);   //读取引脚状态      

	gpio_ic_set(keydesc->gpio_base_addr , GPIO_IC,keydesc->bit, 1);	//清中断

	if (pinval){
		*(key_val+keydesc->number) = (unsigned char)(0x80 | keydesc->key_val);
	}else{
		*(key_val+keydesc->number) = (unsigned char)keydesc->key_val;
	}

	mdelay(50);
    ev_press = 1;                  // 表示中断发生了
    wake_up_interruptible(&button_waitq);   // 唤醒休眠的进程
	
	kill_fasync (&button_async, SIGIO, POLL_IN);   //发送信号
	
	return IRQ_RETVAL(IRQ_HANDLED);
}

/*********************************************************************************************************
*功能：模块打开函数
*参数：
*返回值：无
**********************************************************************************************************/
static int btn_drv_open(struct inode *inode, struct file *file)
{
	unsigned int i;
	int err = -1;

	gpio_reg_group_set();
	key_val = (unsigned char *)kmalloc(sizeof(keys_desc)/sizeof(keys_desc[0]),GFP_KERNEL);
	if(NULL == key_val){
		printk(KERN_ERR"kmalloc fail!");
		return -EBUSY;
	}
	
	for(i=0; i<sizeof(keys_desc)/sizeof(keys_desc[0]); i++)
	{
		err = request_irq(keys_desc[i].irq, buttons_irq, IRQF_SHARED, keys_desc[i].name,(void *)&keys_desc[i]);
		if(err){//如果注册中断失败，则退出
			printk(KERN_ERR" device is busy!irq = %d\n", err); 
			return -EBUSY;
		}

		if(gpio_get_value(keys_desc[i].gpio_base_addr, GPIO_DATA, keys_desc[i].bit))
			*(key_val+i) = 0x80 |  keys_desc[i].key_val;
		else 
			*(key_val+i) = keys_desc[i].key_val;
	}
	
	return 0;
}

/*********************************************************************************************************
*功能：模块读取函数
*参数：
*返回值：无
**********************************************************************************************************/
ssize_t btn_drv_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	if (size != sizeof(keys_desc)/sizeof(keys_desc[0]))
		return -EINVAL;

	if(file->f_flags & O_NONBLOCK){       /* 非 阻塞操作 */  
		copy_to_user(buf, key_val, sizeof(keys_desc)/sizeof(keys_desc[0]));
	}else{
		//如果没有按键动作, 休眠
		wait_event_interruptible(button_waitq, ev_press);

		//如果有按键动作, 返回键值
		copy_to_user(buf, key_val, sizeof(keys_desc)/sizeof(keys_desc[0]));
		ev_press = 0;
	}
	
	return 1;
}

/*********************************************************************************************************
*功能：模块关闭函数
*参数：
*返回值：无
**********************************************************************************************************/
int btn_drv_close(struct inode *inode, struct file *file)     //出链，禁止中断
{
	unsigned int i;

	for(i=0; i<sizeof(keys_desc)/sizeof(keys_desc[0]); i++)
	{
		free_irq(keys_desc[i].irq, (void *)&keys_desc[i]);
	}
	kfree(key_val);
	return 0;
}

/*********************************************************************************************************
*功能：模块软件轮循函数
*参数：
*返回值：无
**********************************************************************************************************/
static unsigned btn_drv_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	poll_wait(file, &button_waitq, wait); // 不会立即休眠，只是把进程挂到队列里面去

	if (ev_press)                          //判断是否有数据返回。有的话进行赋值，没有的话休眠
		mask |= POLLIN | POLLRDNORM;    //返回位掩码, 它描述哪个操作可马上被实现。

	return mask;
}

/*********************************************************************************************************
*功能：模块异步通知函数
*参数：
*返回值：无
**********************************************************************************************************/
static int btn_drv_fasync (int fd, struct file *filp, int on)   
{
	printk("driver: btn_drv_fasync\n");             //为了说明次函数被调用增加一条打印语句
	return fasync_helper (fd, filp, on, &button_async); //初始化定义的结构体
}


static struct file_operations btn_drv_fops = {
    .owner   =  THIS_MODULE,    // 这是一个宏，推向编译模块时自动创建的__this_module变量
    .open    =  btn_drv_open,     
	.read	 =	btn_drv_read,
	.release =  btn_drv_close,
	.poll    =  btn_drv_poll,	//用户程序使用select调用的时候才会用到poll
	.fasync	 =  btn_drv_fasync,	//用户程序用异步通知的时候才会用到fasync
};


int major;
/*********************************************************************************************************
*功能：模块加载函数
*参数：
*返回值：无
**********************************************************************************************************/
static int btn_drv_init(void)
{

	gpio_mux_ctrl_set();	//引脚复用设置
	major = register_chrdev(0, "btn_drv", &btn_drv_fops);	//自动分配主设备号

	btndrv_class = class_create(THIS_MODULE, "btn_drv");

	btndrv_dev = device_create(btndrv_class, NULL, MKDEV(major, 0), NULL, "buttons"); // 设备节点 /dev/buttons


	return 0;
}

/*********************************************************************************************************
*功能：模块卸载函数
*参数：
*返回值：无
**********************************************************************************************************/
static void btn_drv_exit(void)
{
	unregister_chrdev(major, "btn_drv");
	device_unregister(btndrv_dev);
	class_destroy(btndrv_class);
}


module_init(btn_drv_init);
module_exit(btn_drv_exit);
MODULE_DESCRIPTION("Button Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hisilicon");
