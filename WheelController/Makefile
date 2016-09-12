CC=gcc
CXX=g++
CXXFLAGS = -g -Wall

.PHONY = all

# $@ : the left side of the :
# $^ : the right side of the :
# $< : first item in the dependencies list
OBJS = test_serialAPI.o WheelController.o Gear.o SerialPortController.o

all: $(OBJS)
	$(CXX) -o serialAPI $^

#%.o: %.c %.h
#	$(CXX) $(CXXFLAGS) -c $< -o $@ 
test_serialAPI.o: test_serialAPI_4timer.cpp WheelController.o
	$(CXX) $(CXXFLAGS) -c $< -o $@ 

WheelController.o: WheelController.cpp WheelController.h Gear.o SerialPortController.o
	$(CXX) $(CXXFLAGS) -c $< -o $@ 

Gear.o: Gear.cpp Gear.h
	$(CXX) $(CXXFLAGS) -c $< -o $@ 

SerialPortController.o: SerialPortController.cpp SerialPortController.h
	$(CXX) $(CXXFLAGS) -c $< -o $@ 

.PHONY: clean
clean: 
	rm -f *.o
