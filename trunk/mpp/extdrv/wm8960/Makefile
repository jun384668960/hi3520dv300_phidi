
ifeq ($(PARAM_FILE), )
	PARAM_FILE:=../../Makefile.param
	include $(PARAM_FILE)
endif
			
obj-m := wm_8960.o
wm_8960-y += wm8960.o

ifeq ($(CONFIG_GPIO_I2C), y)
    EXTRA_CFLAGS += -DHI_GPIO_I2C
	EXTRA_CFLAGS+=-I$(PWD)/../gpio-i2c-ex
else
	EXTRA_CFLAGS += -DHI_I2C
	EXTRA_CFLAGS+=-I$(PWD)/../hi_i2c
endif

EXTRA_CFLAGS += -I$(REL_INC)
EXTRA_CFLAGS += $(DRV_CFLAGS)
EXTRA_CFLAGS += -D$(HI_FPGA)
# EXTRA_CFLAGS += -std=c99
EXTRA_CFLAGS += -std=gnu99

default:
	@make -C $(LINUX_ROOT) M=$(PWD) modules 
#	cp wm_8960.ko ../../../user 
clean: 
	@make -C $(LINUX_ROOT) M=$(PWD) clean 
#	@-rm ../../../user/wm_8960.ko
