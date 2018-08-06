CROSS := arm-hisiv300-linux-
STRIP := $(CROSS)strip

rel:
	-@mkdir output
	@cd mpp/sample/phidi;   make
	@cd record;    make
	@cd output/;    $(STRIP) phidi mp4file
#	@cd mpp/extdrv/wm8960/;    make
#	@cd mpp/extdrv/it6801/;    make
	
clean:
	-@rm -rf output/
	@cd mpp/sample/phidi;   make clean
	@cd record;    make clean
#	@cd mpp/extdrv/wm8960/;    make clean
#	@cd mpp/extdrv/it6801/;    make clean
