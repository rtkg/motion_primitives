
####### Output directories

ODIR = obj
EDIR = bin
SDIR = src

####### Define ACADO related variables

LOCAL_PATH_PREFIX = /home/rkg/Data/Programs/ACADOtoolkit-1.0.2613beta
include ${LOCAL_PATH_PREFIX}/include/acado/include.mk

####### Compiler, tools and options

CC       = gcc
CXX      = g++
CFLAGS   = -pipe -Wall -W -O2 -fPIC 
CXXFLAGS =  ${CPP_GLOBAL_FLAGS} ${NO_PARENTHESES_WARNING}  #use the ACADO flags
AR       = ar cqs 
DEL_FILE = rm -f

####### Include directories for external library headers

INCPATH  = ${HEADER_PATHS} #add ACADO headers

####### External libraries

LIBS = ${TOOLKIT_LIBS} #add ACADO libs

####### Files for compiling the code

DS_HEADERS = 	

DS_SOURCES = 	./src/fit2OrderSystemMakkar.cpp 

DS_OBJECTS =	./obj/fit2OrderSystemMakkar.o

DS_EXECS   =    ./bin/fit2OrderSystemMakkar

####### Implicit rules

.SUFFIXES: .c .o .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o $@ $<

####### Build rules

all: fit2OrderSystemMakkar

fit2OrderSystemMakkar: ./obj/fit2OrderSystemMakkar.o
	$(CXX)  $< $(INCPATH) $(LIBS) -o $(EDIR)/fit2OrderSystemMakkar

####### Clean up	

clean:
	-$(DEL_FILE) $(DS_OBJECTS) $(DS_EXECS) 
	-$(DEL_FILE) *~ core *.core

####### Compile the source code

$(ODIR)/fit2OrderSystemMakkar.o: $(SDIR)/fit2OrderSystemMakkar.cpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(ODIR)/fit2OrderSystemMakkar.o $(SDIR)/fit2OrderSystemMakkar.cpp
















