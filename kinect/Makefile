all: video

CFLAGS=-fPIC -g -Wall `pkg-config --cflags opencv --cflags libusb-1.0`
LIBS = `pkg-config --libs opencv`
INCLUDE = -I/usr/local/include/libfreenect
FREE_LIBS = -L/usr/local/lib -lfreenect

video:  video.cpp
	g++ $(INCLUDE) $(CFLAGS) $? -o $@  $(LIBS) $(FREE_LIBS)

%.o: %.cpp
	g++ -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o test
