#project : v2x_appv_txv Makefile
#version : 1.0.0
#author  : wuhh
#date    : 2017-5-2
CPP = /home/genvict/workspace/buildroot-2014.05/output/host/usr/bin/arm-buildroot-linux-uclibcgnueabi-g++
STRIP = /home/genvict/workspace/buildroot-2014.05/output/host/usr/bin/arm-buildroot-linux-uclibcgnueabi-strip

DIR_OBJ = ./obj
DIR_BIN = ./bin
DIR_SRC = ./src
DIR_INC = ./inc
DIR_LIB = ./lib

INCFLAGS = -I$(DIR_INC) -I./live/UsageEnvironment/include -I./live/groupsock/include -I./live/liveMedia/include -I./live/BasicUsageEnvironment/include
CFLAGS = -Wall $(INCFLAGS) -g -DBSD=1 -I. -O2 -DSOCKLEN_T=socklen_t -DNO_SSTREAM=1 -D_LARGEFILE_SOURCE=1 -D_FILE_OFFSET_BITS=64
LDFLAGS = -L./live/liveMedia -L./live/groupsock -L./live/BasicUsageEnvironment -L./live/UsageEnvironment -L$(DIR_LIB) -lliveMedia -lgroupsock -lBasicUsageEnvironment -lUsageEnvironment -pthread -lm -lv2xgeneralcpp

vpath %.h $(DIR_INC)
vpath %.hh ./live/UsageEnvironment/include 
vpath %.hh ./live/groupsock/include 
vpath %.hh ./live/liveMedia/include 
vpath %.hh ./live/BasicUsageEnvironment/include
vpath %.cpp $(DIR_SRC)
vpath %.o $(DIR_OBJ)

SOURCES = $(wildcard $(DIR_SRC)/*.cpp)  
OBJS=$(patsubst %.cpp,%.o,$(notdir $(SOURCES)))

TARGET = v2x_appv_txv
BIN_TARGET = ${DIR_BIN}/${TARGET}

all: ${BIN_TARGET}

$(BIN_TARGET): $(OBJS)
	$(CPP) $(addprefix $(DIR_OBJ)/,$(OBJS)) $(LDFLAGS) -o $@
	$(STRIP) $(BIN_TARGET)
	
%.o:%.cpp
	$(CPP) $(CFLAGS) -c $< -o $(DIR_OBJ)/$@
	
.PHONY:clean
clean:
	rm ${DIR_OBJ}/*.o $(BIN_TARGET)
