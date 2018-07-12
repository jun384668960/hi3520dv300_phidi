ifeq ($(PARAM_FILE), )
        PARAM_FILE:=../../Makefile.param
        include $(PARAM_FILE)
endif

obj-m := it6801drv.o
it6801drv-objs := it6801.o it680x.o IO.o 

default:
	@make -C $(LINUX_ROOT) M=$(PWD) modules

clean: 
	@make -C $(LINUX_ROOT) M=$(PWD) clean


