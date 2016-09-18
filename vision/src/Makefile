all: test

CFLAGS=-fPIC -g `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
#INCLUDE = -I/usr/local/include/libfreenect
#FREE_LIBS = -L/usr/local/lib -lfreenect

# OBJS = WheelController.o Gear.o SerialPortController.o test_serialAPI_4timer.o

.PHONY =  cll

# test: second_rotation_2.cpp
test: road.cpp WheelController.cpp Gear.cpp SerialPortController.cpp
# serial: 
# test: second.cpp
# test: second_puzzle.cpp
# test: color_correct.cpp
	$(CXX) $(CFLAGS) $? -o $@  $(LIBS)

%.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o test