
ifeq ($(PARAM_FILE), )
		PARAM_FILE:=../../Makefile.param
		include $(PARAM_FILE)
endif

obj-m := btn_drv.o
hi_drv-y += btn_drv.o

EXTRA_CFLAGS += -I$(REL_INC)
EXTRA_CFLAGS += $(DRV_CFLAGS)
default:	
	$(CC) -g -Wall -o btn_drv_test btn_drv_test.c
	@$(MAKE) -C $(LINUX_ROOT) M=$(PWD) modules 
	@rm -f *.o modules.* *.symvers *.mod.c
	@mv -f *test ./app/
	@mv -f *.ko ./ko/
clean:
	@make -C $(LINUX_ROOT) M=$(PWD) clean 
	@rm -f ./app/* *.bak

