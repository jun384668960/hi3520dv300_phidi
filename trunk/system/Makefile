CROSS := arm-hisiv300-linux-
CC := $(CROSS)gcc
CFLAG := -Wno-write-strings -lpthread \
		-I../utils -I./include \
		
BINS := system

SRCS := $(wildcard ./src/*.c)
SRCS += $(wildcard ../utils/*.c)
OBJS := $(wildcard ./src/*.o ../utils/*.o)

all: $(BINS)

$(BINS):
	$(CC) $(SRCS) -o $(BINS) $(CFLAG)
	
clean:
	@rm -rf $(BINS)
	@rm -rf $(OBJS)

