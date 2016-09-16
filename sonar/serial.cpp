#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>

int main(void){
  int fd; // file descriptor of port.
  //fd = open("/dev/cu.usbmodem1411", O_RDWR | O_NOCTTY | O_NDELAY);
  //fd = open("/dev/cu.usbmodem1411", O_RDWR | O_NOCTTY);
  fd = open("/dev/cu.wchusbserial1410", O_RDWR | O_NOCTTY);
  //fd = open("/dev/cu.wchusbserial1410", O_RDWR | O_NONBLOCK);

  if (fd == -1) {
    // Could not open the port.
    perror("open_port: Unable to open port");
  } else 
    fcntl(fd, F_SETFL, 0);

  

  struct termios options;

  // Get the current options for the port...
  if(tcgetattr(fd, &options) < 0){
    perror("gg");
  }
  
  // Set the baud rates to 115200
  cfsetispeed(&options, (speed_t)B115200);
  cfsetospeed(&options, (speed_t)B115200);

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

  if( tcsetattr(fd, TCSANOW, &options) < 0) {
    perror("init_serialport: Couldn't set term attributes");
    printf("errno : %i", errno);
    return -1;
  }

  if (tcgetattr(fd, &options) < 0) {
    perror("serialport_init: Couldn't get term attributes");
    return -1;
  }
  printf("baud rate : %lu\n",cfgetispeed(&options));

  usleep(2*1000*1000);
  
  char dummy[] = {'A'};
  char data[400*4];
  int n = write(fd, "A", 1);
  printf("Wrote %i\n", n);
  //printf("lol\n");
  //usleep(1000);
  n = read(fd, data, 400*2);
  printf("Read %i\n", n);

  union u_tag {
   unsigned char b[2];
   int val;
  } u;


  u.b[1] = data[0];
  u.b[0] = data[1];

  printf("%i\n", u.ival);

  
  close(fd);
  return 0;
}