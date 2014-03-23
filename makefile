CC=g++
CCFLAGS=`pkg-config --cflags --libs opencv`
OS=object_segmentation
OT=object_track
OD=odometry
MODULES=$(OS) $(OT) $(OD)
BUILD=build

all: OS OT OD
all1: create OS OT OD

create:
	mkdir build && cp $(OT)/mask.png $(BUILD)
OS: 	OS.o
	$(CC) $(CCFLAGS) $(BUILD)/OS.o -o $(BUILD)/OS

OT: 	OT.o
	$(CC) $(CCFLAGS) $(BUILD)/OT.o -o $(BUILD)/OT

OD: 	OD.o
	$(CC) $(CCFLAGS) $(BUILD)/OD.o -o $(BUILD)/OD

OS.o: 	
	$(CC) -c $(CCFLAGS) $(OS)/farneback.cpp -o $(BUILD)/OS.o

OT.o:
	$(CC) -c $(CCFLAGS) $(OT)/lk_based.cpp -o $(BUILD)/OT.o	

OD.o:
	$(CC) -c $(CCFLAGS) $(OD)/position.cpp -o $(BUILD)/OD.o

clean:
	rm -rf build
