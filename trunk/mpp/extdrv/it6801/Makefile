ifeq ($(PARAM_FILE), )
        PARAM_FILE:=../../Makefile.param
        include $(PARAM_FILE)
endif

obj-m := it6801drv.o
it6801drv-objs := it6801.o it680x.o IO.o srt.o

default:
	@make -C $(LINUX_ROOT) M=$(PWD) modules
	-@[ -d "$(SDK_PATH)/output/" ] && cp it6801drv.ko $(SDK_PATH)/output/

clean: 
	@make -C $(LINUX_ROOT) M=$(PWD) clean


