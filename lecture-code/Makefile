CC = gcc
CFLAGS = -Wall -g

all: test-wheel trapezoidal kinematics ptctrl splinebase splinecurve

test-wheel: wheel.o test-wheel.o Makefile
	$(CC) $(CFLAGS) -o test-wheel test-wheel.o wheel.o -lm

trapezoidal: trapezoidal.o Makefile
	$(CC) $(CFLAGS) -o trapezoidal trapezoidal.o -lm

kinematics: kinematics.o Makefile
	$(CC) $(CFLAGS) -o kinematics kinematics.o -lm

ptctrl: ptctrl.o Makefile
	$(CC) $(CFLAGS) -o ptctrl ptctrl.o -lm

splinebase: splinebase.o Makefile
	$(CC) $(CFLAGS) -o splinebase splinebase.o -lm

splinecurve: splinecurve.o Makefile
	$(CC) $(CFLAGS) -o splinecurve splinecurve.o -lm

wheel.o test-wheel.o: wheel.h Makefile
