all: test

CFLAGS=-fPIC -g `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
#INCLUDE = -I/usr/local/include/libfreenect
#FREE_LIBS = -L/usr/local/lib -lfreenect

test:  second.cpp
	$(CXX) $(CFLAGS) $? -o $@  $(LIBS)

%.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o test