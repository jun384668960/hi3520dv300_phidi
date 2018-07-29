CROSS := arm-hisiv300-linux-
STRIP := $(CROSS)strip

rel:
	@cd mpp/sample/phidi;   make
	@cd record;    make
	$(STRIP) phidi
	$(STRIP) mp4file
	
clean:
	@cd mpp/sample/phidi;   make clean
	@cd record;    make clean