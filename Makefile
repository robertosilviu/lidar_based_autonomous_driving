#-----------------------------------------------------------------------
# This is the Makefile for compilation of Autonomous Driving project
#-----------------------------------------------------------------------

#-----------------------------------------------------------------------
# Target file to be compiled by default
#-----------------------------------------------------------------------
MAIN = autonomous_driving
#-----------------------------------------------------------------------
# CC is the compiler to be used
#-----------------------------------------------------------------------
CC = gcc
#-----------------------------------------------------------------------
# CFLAGS are the options passed to the compiler
#-----------------------------------------------------------------------
CFLAGS = -Wall -lpthread -lrt
#-----------------------------------------------------------------------
# Custom libraries linking
#-----------------------------------------------------------------------
PTASK_LIB = ptask
TLIB = tlib
QLEARN_LIB = qlearn
KOHONEN_LIB = kohonen
OBJS = $(MAIN).o $(PTASK_LIB).o $(TLIB).o $(QLEARN_LIB).o $(KOHONEN_LIB).o
#-----------------------------------------------------------------------
# LIBS are the external libraried to be used
#-----------------------------------------------------------------------
LIBS = -lpthread -lrt -lm `allegro-config --libs`
#-----------------------------------------------------------------------
# Dependencies
#-----------------------------------------------------------------------
$(MAIN): $(OBJS)
	$(CC) $(CFLAGS) -o $(MAIN) $(OBJS) $(LIBS)

MAIN_PATH = src/$(MAIN)
$(MAIN).o: $(MAIN_PATH).c
	$(CC) $(CFLAGS) -c $(MAIN_PATH).c

PTASK_PATH = libs/ptask/$(PTASK_LIB)
$(PTASK_LIB).o: $(PTASK_PATH).c
	$(CC) $(CFLAGS) -c $(PTASK_PATH).c

TLIB_PATH = libs/tlib/$(TLIB)
$(TLIB).o: $(TLIB_PATH).c
	$(CC) $(CFLAGS) -c $(TLIB_PATH).c

QLEARN_PATH = libs/qlearn/$(QLEARN_LIB)
KOHONEN_PATH = libs/qlearn/$(KOHONEN_LIB)
$(QLEARN_LIB).o: $(QLEARN_PATH).c $(KOHONEN_PATH).c
	$(CC) $(CFLAGS) -c $(QLEARN_PATH).c $(KOHONEN_PATH).c

clean:
	rm -rf *.o $(MAIN)