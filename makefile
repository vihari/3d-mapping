CC=g++
CCFLAGS=`pkg-config --cflags --libs opencv`
OS=object_segmentation
OT=object_track
OD=odometry
MODULES=$(OS) $(OT) $(OD)
BUILD=build

all: OS OT OD

OS: 	OS.o
	$(CC) $(CCFLAGS) OS.o -o $(BUILD)/OS

OT: 	OT.o
	$(CC) $(CCFLAGS) OT.cpp -o $(BUILD)/OT

OD: 	OD.o
	$(CC) $(CCFLAGS) OD.cpp -o $(BUILD)/OD

OS.o: 	
	$(CC) $(CCFLAGS) $(OS)/farneback.cpp -o $(BUILD)/OS.o

OT.o:
	$(CC) $(CCFLAGS) $(OT)/lk_based.cpp -o $(BUILD)/OT.o	

OD.o:
	$(CC) $(CCFLAGS) $(OD)/position.cpp -o $(BUILD)/OD.o

clean:
	rm -rf build
