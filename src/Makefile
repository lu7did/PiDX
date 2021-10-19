all: ../bin/PiDX 

OSC_CFLAGS=-DOSCILLATOR_Z -DOSCILLATOR_D

CXXFLAGS = -std=c++11 -Wall -g -O2 -Wno-unused-variable
LDFLAGS = /home/pi/librpitx/src/librpitx.a -lm -lrt -lpthread -lwiringPi -lwiringPiDev -lpigpio
CFLAGS  = -Wall -g -O2 -Wno-unused-variable
CXXFLAGS = -std=c++11 -Wall -g -O2 -Wno-unused-variable

CCP = c++
CC = cc
GCC = gcc

../bin/PiDX : PiDX/PiDX.cpp /home/pi/librpitx/src/librpitx.a
	$(CCP) $(OSC_CFLAGS) $(CXXFLAGS) -o ../bin/PiDX ./PiDX/PiDX.cpp   $(LDFLAGS)


clean:
	rm -r  ../bin/PiDX

install: all
	install -m 0755 ../bin/PiDX  /usr/bin
