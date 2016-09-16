#include "SerialPortController.h"
#include <fcntl.h>          // open(), close()
#include <unistd.h>         /* UNIX standard function definitions */
#include <stdio.h>          // prinf()
#include <stdlib.h>         // exit()
#include <termios.h>        // POSIX terminal control definitions
#include <sys/ioctl.h>      // iotcl()


SerialPortController::SerialPortController(char* filename){
  if ((_device_handle = open(filename, O_RDWR | O_NOCTTY)) < 0){
    printf("Error: Couldn't open device %s\n", filename);
    exit(1);
  }
  fcntl(_device_handle, F_SETFL, 0);

  struct termios options;

  // Get the current options for the port...
  if(tcgetattr(_device_handle, &options) < 0){
    perror("Error: Couldn't get device serial options\n");
  }
  
  // Set the baud rates to 115200
  cfsetispeed(&options, (speed_t)B38400);
  cfsetospeed(&options, (speed_t)B38400);

  cfmakeraw(&options);

  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~CRTSCTS; 
  options.c_cflag |= CLOCAL | CREAD;

  options.c_iflag |= IGNPAR | IGNCR;
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_lflag |= ICANON;
  options.c_oflag &= ~OPOST;

  options.c_cc[VMIN] = 1;
  options.c_cc[VTIME] = 10;

  if( tcsetattr(_device_handle, TCSANOW, &options) < 0) {
    perror("init_serialport: Couldn't set term attributes");
    exit(1);
  }

  if (tcgetattr(_device_handle, &options) < 0) {
    perror("serialport_init: Couldn't get term attributes");
    exit(1);
  }

  printf("baud rate : %lu\n",cfgetispeed(&options));

  sleep(1);
}

SerialPortController::~SerialPortController(){
  close(_device_handle);
}

// public functions
void SerialPortController::sendByte(uint8_t* data, int len){
  int n = write(_device_handle, data, len);
  if (n != len) {
    printf("Did not send all requested bytes.(%i/%i)\n", n, len);
    exit(1);
  }
}

// private functions

