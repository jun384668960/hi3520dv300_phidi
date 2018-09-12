// #include "EnDe.h"
#include "typedef.h"
#include <linux/gpio.h>
#include "srt.h"

#define ERROR_CODE_READ_ADDR (-1)
#define ERROR_CODE_WRITE_ADDR (-2)
#define ERROR_CODE_WRITE_DATA (-3)
#define ERROR_CODE_TRUE (0)

extern const char gpio_fm_i2c_scl_pin;
extern const char gpio_fm_i2c_sda_pin;

void HZ008DelayMs(uint data);

#define HZ008_SDA	gpio_fm_i2c_sda_pin
#define HZ008_SCL	gpio_fm_i2c_scl_pin

#define I2C_WRITE	0
#define I2C_READ	1

int GPIO_ReadIO(unsigned gpio)
{
	return gpio_get_value(gpio);
}

void GPIO_WriteIO(int value, unsigned gpio)
{
	gpio_set_value(gpio, value);
}

int GPIO_InitIODir(int value, unsigned gpio)
{
	return gpio_direction_output(gpio, value);
}

int GPIO_ModeSetup(unsigned gpio, int value)
{
	return 0;
}

#define SDA_GPIO_M	GPIO_ModeSetup(HZ008_SDA, 0)
#define SCL_GPIO_M	GPIO_ModeSetup(HZ008_SCL, 0)
#define SDA_HIGH	GPIO_WriteIO(1, HZ008_SDA)
#define SDA_LOW		GPIO_WriteIO(0, HZ008_SDA)
#define SCL_HIGH	GPIO_WriteIO(1, HZ008_SCL)
#define SCL_LOW		GPIO_WriteIO(0, HZ008_SCL)
#define SDA_IN		GPIO_InitIODir(0, HZ008_SDA)
#define SDA_OUT		GPIO_InitIODir(1, HZ008_SDA)
#define SCL_OUT		GPIO_InitIODir(1, HZ008_SCL)
#define SDA_DETECT	GPIO_ReadIO(HZ008_SDA)

#define I2C_DELAY		HZ008Delay(150) 
#define I2C_DELAY_LONG	HZ008DelayMs(50)

void i2c_init(void)
{
	HZ008DelayMs(100);
}

void HZ008DelayMs_51(uint data)
{
	unsigned char i;
	while(data--)
	{
		for(i=0;i<5000;i++){}  
	}
}

void HZ008DelayMs(uint ms)
{
	msleep(ms);
}

void HZ008Delay(uint data)
{
	unsigned char i;
	while(data--)
	{
	}
}

// I2C START
#define PRINTFMODE		1
void HZ008_i2c_start(void)
{
	//SDA_GPIO_M;
	//SCL_GPIO_M;
	SDA_HIGH;	
	SCL_HIGH;	//just in case default output value is 0, to avoid unexcept falling edge while set I/O as output
	
	SDA_OUT;
	SCL_OUT;

	SDA_HIGH;
	I2C_DELAY;
	SCL_HIGH;
	I2C_DELAY;
	I2C_DELAY;
	
	SDA_LOW;
	I2C_DELAY;
	I2C_DELAY;
	SCL_LOW;
	I2C_DELAY;
}

void HZ008_i2c_stop(void)
{

	SDA_OUT;
	SCL_OUT;
	
	SDA_LOW;
	I2C_DELAY;
	I2C_DELAY;
	SCL_HIGH;
	I2C_DELAY;
	I2C_DELAY;
	
	SDA_HIGH;
	I2C_DELAY;
	I2C_DELAY;
	I2C_DELAY;
	I2C_DELAY;
}

unsigned char HZ008_i2c_write_byte(unsigned char data)
{
	unsigned char i, ack;
	// kal_bool ret;

	SDA_OUT;
	I2C_DELAY;   

	for(i = 0; i< 8; i++)
	{
		if( (data << i) & 0x80) 
		{
			SDA_HIGH;
			I2C_DELAY;
		}
		else 
		{
			SDA_LOW;
			I2C_DELAY;
		}
		SCL_HIGH;
		I2C_DELAY;
		SCL_LOW;
		I2C_DELAY;
	}
	
	SDA_IN;
	I2C_DELAY;  
	SCL_HIGH;
	I2C_DELAY;
	ack = GPIO_ReadIO(HZ008_SDA); /// ack    

	
	printk(" _i2c_write_byte, ack = %d \r\n", ack);
	I2C_DELAY;
	SCL_LOW;
	SDA_OUT;
	return ack;
}

unsigned char HZ008_i2c_read_byte_ack(void)
{
	unsigned char i, data;
	unsigned char v_return;
	data = 0;

	SDA_IN;
	I2C_DELAY;   
	for(i = 0; i< 8; i++)
	{
		data <<= 1;
		I2C_DELAY;
		SCL_HIGH;
		I2C_DELAY;
		data |= GPIO_ReadIO(HZ008_SDA);
                I2C_DELAY;
		SCL_LOW;
		I2C_DELAY;
	}
	SDA_OUT;
	GPIO_WriteIO(0, HZ008_SDA);
	I2C_DELAY;
	GPIO_WriteIO(1, HZ008_SCL);
	v_return = (unsigned char)data&0xFF;
	I2C_DELAY;
	GPIO_WriteIO(0, HZ008_SCL);
	I2C_DELAY;
	
	return v_return;
}

unsigned char HZ008_i2c_read_byte_noack(void)
{
	unsigned char i, data;
	unsigned char v_return;
	data = 0;

	SDA_IN;
	I2C_DELAY;   
	for(i = 0; i< 8; i++)
	{
		data <<= 1;
		I2C_DELAY;
		SCL_HIGH;
		I2C_DELAY;
		data |= GPIO_ReadIO(HZ008_SDA);
		SCL_LOW;
		I2C_DELAY;
	}
	SDA_OUT;
	GPIO_WriteIO(1, HZ008_SDA);
	I2C_DELAY;
	GPIO_WriteIO(1, HZ008_SCL);
	v_return = (unsigned char)data&0xFF;
	I2C_DELAY;
	GPIO_WriteIO(0, HZ008_SCL);
	I2C_DELAY;
	
	return v_return;
}

unsigned char HZ008_i2c_write(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo)
{
	unsigned char i;

	HZ008_i2c_start();
	I2C_DELAY_LONG;
	if(HZ008_i2c_write_byte(device_addr&0xFF))   //
	{
		HZ008_i2c_stop();
		#if PRINTFMODE
		printk("\n\rWRITE I2C : Write Error - Device Addr");
		#endif
		return ERROR_CODE_WRITE_ADDR;
	}
	if(HZ008_i2c_write_byte(sub_addr)) 
	{
		HZ008_i2c_stop();
		#if PRINTFMODE
		printk("\n\rWRITE I2C : Write Error - Sub Addr");
		#endif
		return ERROR_CODE_WRITE_ADDR;
	}
	for(i = 0; i<ByteNo; i++) 
	{
		if(HZ008_i2c_write_byte(buff[i])) 
		{
			HZ008_i2c_stop();
			#if PRINTFMODE
			printk("\n\rWRITE I2C : Write Error - TX Data");
			#endif
			return ERROR_CODE_WRITE_DATA;
		}
	}
	I2C_DELAY;
	HZ008_i2c_stop();
	I2C_DELAY_LONG;
	return ERROR_CODE_TRUE;
}

unsigned char HZ008_i2c_read(unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo)
{
	unsigned char i;

	#if PRINTFMODE
	printk("\r\n_i2c_read");
	#endif
	HZ008_i2c_start();
	I2C_DELAY_LONG;
	if(HZ008_i2c_write_byte(0xc0)) 
	{
		HZ008_i2c_stop();
		#if PRINTFMODE
		printk("\n\r Read I2C Write Error - device Addr\r\n");
		#endif
		return ERROR_CODE_READ_ADDR;
	}
	if(HZ008_i2c_write_byte(0xa0)) 
	{
		HZ008_i2c_stop();
		#if PRINTFMODE
		printk("\n\rRead I2C Write Error - sub Addr \r\n");
		#endif
		return ERROR_CODE_READ_ADDR;
	}
	HZ008_i2c_start();
	if(HZ008_i2c_write_byte(0xc1)) 
	{
		HZ008_i2c_stop();
		#if PRINTFMODE
		printk("\n\rRead I2C Write Error - sub Addr \r\n");
		#endif
		return ERROR_CODE_READ_ADDR;
	}

	for(i = 0; i<ByteNo; i++) 
	{
	     if(i<ByteNo-1)
		buff[i] = HZ008_i2c_read_byte_ack();
	    else
		buff[i] = HZ008_i2c_read_byte_noack();	
	}
	I2C_DELAY;
	I2C_DELAY_LONG;
	HZ008_i2c_stop();
	I2C_DELAY_LONG;
	return ERROR_CODE_TRUE;
}


#define tmp_sub_addr 0xA0

unsigned char g_PrgramCnt;
unsigned char tmp_dev_addr = 0xC0; //0xC4
//ADD(芯片第二脚)接低电平时，读写地址是C0，如果接高电平时，地址是C4

unsigned char GetRandom(void)
{
	/*unsigned char seed=0;
	
	ADMUX=0x40;
	ADCSRA|=(1<<ADEN)|(1<<ADSC);
	while(ADCSRA&(1<<ADSC))		//wait until ADC done
		;
	seed+=ADC;
	seed+=TCNT3+TCNT0+TCNT1+TCNT2;
	srandom(seed);*/
	return 0;
	// return(rand());
}

static unsigned char EDesEn_Crypt(unsigned char *rx_data, unsigned char *ex_data)
{
	return Amba_app_srt_edesen_crypt(rx_data, ex_data);
}

unsigned char TestProtection(void)
{
	unsigned char error_code, i;
	unsigned char tx_data[8],rx_data[10],ex_data[8];
	unsigned char value=5;
	
	error_code=0;
	i2c_init();
	HZ008DelayMs(200);
	printk("TestProtection begin\r\n");
	printk("tx_data: \r\n");
	for(i=8; i!=0; i--)
	{
		tx_data[8-i]=GetRandom();
		printk("tx_data[%d] = %d \r\n",8-i, tx_data[8-i]);//
	}

	value=HZ008_i2c_write(0XC0, tmp_sub_addr,tx_data , 8);
	printk("value=%x\r\n",value);
	HZ008DelayMs(500);

	HZ008_i2c_read(0XC1, tmp_sub_addr,rx_data, 10);
	printk("rx_data: \r\n");
	for(i = 0; i < 10; i++)
	{
		printk("rx_data[%d] = %d   \r\n",i, rx_data[i]);
	}
	EDesEn_Crypt(rx_data, ex_data);
	printk("ex_data: \r\n");
	for(i = 0; i < 8; i++)
	{
		printk("ex_data[%d] = %d \r\n",i, ex_data[i]);
	}
	for(i=0;i<8;i++)
	{
		if(tx_data[i]!=ex_data[i])
		error_code=1;
	}
	if(error_code)
	{
		printk(" EDesEn_Crypt failed \r\n");
		return 0;
	}
	else
	{
		printk(" EDesEn_Crypt pass \r\n");
		return 1;
	}
	
}

#if 0
void HZ008PowerOnCheck(void)
{
	unsigned char i = 0, v_result = 1;
	unsigned char *ptr = NULL;

		v_result = TestProtection();
		HZ008DelayMs(100);
		if(1 == v_result)
		{
			return;
		}
		else
		{
			while(1)
			{
				stop();
			}
		}
		HZ008DelayMs(50);
}



#endif
/*
int main(void)
{
	i2c_init();
	tmp_dev_addr=0xc0;
	HZ008DelayMs(20);
	TestProtection();
	while(1)
	{
		g_PrgramCnt++;
	}

}
*/