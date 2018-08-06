CROSS := arm-hisiv300-linux-
STRIP := $(CROSS)strip

rel:
	-@mkdir output
	@cd mpp/sample/phidi;   make
	@cd record;    make
	@cd mpp/extdrv/wm8960/;    make
	@cd mpp/extdrv/it6801/;    make
	@cd output/;    $(STRIP) phidi mp4file
	-@./cp_to_rftp.sh

phidi:
	-@mkdir output
	@cd mpp/sample/phidi/;    make
	-@./cp_to_rftp.sh
	
wm8960:
	-@mkdir output
	@cd mpp/extdrv/wm8960/;    make
	-@./cp_to_rftp.sh
	
it6801:
	-@mkdir output
	@cd mpp/extdrv/it6801/;    make
	-@./cp_to_rftp.sh

	
clean:
	-@rm -rf output/
	@cd mpp/sample/phidi;   make clean
	@cd record;    make clean
	@cd mpp/extdrv/wm8960/;    make clean
	@cd mpp/extdrv/it6801/;    make clean
