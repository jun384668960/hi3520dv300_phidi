CROSS := arm-hisiv300-linux-
CC := $(CROSS)g++
CFLAG := -Wno-write-strings -lpthread -lmp4 -L./lib \
		-I../common -I../utils -I./include \
		
BINS := mp4file

SRCS := $(wildcard ./src/*.cpp)
SRCS += $(wildcard ../utils/*.c)
OBJS := $(wildcard ./*.o ./src/*.o ../utils/*.o)

all: $(BINS)

$(BINS):
	$(CC) main.cpp $(SRCS) -o $(BINS) $(CFLAG)
	-@[ -d "../output" ] && cp $(BINS) ../output
	
clean:
	@rm -rf $(BINS)
	@rm -rf $(OBJS)
