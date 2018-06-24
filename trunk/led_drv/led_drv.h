#ifndef __LED_DRV_H__
#define __LED_DRV_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

/**************引脚复用寄存器************/
    #define MUX_CTRL_REG_BASE               0x120F0000
    #define GPIO10_4_MUX_CTRL_REG_OFFSET    0x188

/***********GPIO基址********************/
    #define GPIO13_BASE	0x12220000
    #define GPIO12_BASE	0x12210000
    #define GPIO11_BASE	0x12200000
    #define GPIO10_BASE	0x121F0000
    #define GPIO09_BASE	0x121E0000
    #define GPIO08_BASE	0x121D0000
    #define GPIO07_BASE	0x121C0000
    #define GPIO06_BASE	0x121B0000
    #define GPIO05_BASE	0x121A0000
    #define GPIO04_BASE	0x12190000
    #define GPIO03_BASE	0x12180000
    #define GPIO02_BASE	0x12170000
    #define GPIO01_BASE	0x12160000
    #define GPIO00_BASE	0x12150000

/**************GPIO相关寄存器*************/
    #define GPIO_DATA	0x3fc
    #define GPIO_DIR	0x400
    #define GPIO_IS		0x404
    #define GPIO_IBE	0x408
    #define GPIO_IEV	0x40c
    #define GPIO_IE		0x410
    #define GPIO_RIS	0x414
    #define GPIO_MIS	0x418
    #define GPIO_IC		0x41c

/*************GPIO中断号*****************/
	#define IRQ_GPIO10           	91

	#define IRQ_GPIO10_4    	IRQ_GPIO10

    //读写写寄存器
    #define  GPIO_WRITE_REG(base_addr,offset_addr, value) (*(volatile unsigned int *)((unsigned int)ioremap_nocache(base_addr,0x800) + offset_addr) = (value))
    #define  GPIO_READ_REG(base_addr,offset_addr)         (*(volatile unsigned int *)((unsigned int)ioremap_nocache(base_addr,0x800) + offset_addr))


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif	/* __BTN_DRV_H__ */