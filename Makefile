CC = gcc -I/usr/include/libusb-1.0
GXX = g++
CXXFLAGS      = -pipe -O2 -Wall -W -D_REENTRANT -lusb-1.0 -lm
INCPATH = -I. -I/usr/include/libusb-1.0

all:  Palettes.o flir8i.o flir8i


Palettes.o: Palettes.cpp Palettes.h
	${CXX} -c ${CXXFLAGS} ${INCPATH} -o Palettes.o Palettes.cpp

flir8i.o: flir8i.c

flir8i: flir8i.o
	${CC} -o flir8i Palettes.o flir8i.o -lusb-1.0 -ljpeg -lm -Wall


clean:
	rm -f  Palettes.o flir8i.o flir8i
