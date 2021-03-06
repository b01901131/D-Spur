#include "I2CBusController.h"
#include <fcntl.h>          // open(), close()
#include <unistd.h>         /* UNIX standard function definitions */
#include <linux/i2c-dev.h>  // i2c_smbus_write_byte()
#include <stdio.h>          // prinf()
#include <stdlib.h>         // exit()
#include <sys/ioctl.h>      // iotcl()


I2CBusController::I2CBusController(char* filename){
  if ((_device_handle = open(filename, O_RDWR)) < 0){
    printf("Error: Couldn't open device %d\n", _device_handle);
    exit(1);
  }
  _sl_addr = 255;
}

I2CBusController::~I2CBusController(){
  close(_device_handle);
}

// public functions
void I2CBusController::sendByte(uint8_t sl_addr, uint8_t data){
  if(_sl_addr != sl_addr)
    setSlave(sl_addr);

  i2c_smbus_write_byte(_device_handle, data);
}

void I2CBusController::sendBytes(uint8_t sl_addr, uint8_t* data, int len){
  if(_sl_addr != sl_addr)
    setSlave(sl_addr);

  for(int i = 0; i < len ; i++){
    i2c_smbus_write_byte(_device_handle, *(data+i));
    //usleep(50);
    uint8_t ack = i2c_smbus_read_byte(_device_handle);
    printf("i2c ack:%i\n", ack);
    //if(ack != i+1 && ack != 255){
      //i = ack;
    //}
  }
  //uint8_t ack = i2c_smbus_read_byte(_device_handle);
  //printf("i2c ack:%i\n", ack);
  printf("i2c bus send all requested bytes\n");
  
  //i2c_smbus_write_block_data(_device_handle, 0x0, len, data);

  //int n = write(_device_handle, data, len);
  //if (n != len) {
  //  printf("Did not send all requested bytes.(%i/%i)\n", n, len);
  //  exit(1);
  //}
}
// private functions
void I2CBusController::setSlave(uint8_t sl_addr){
  if (ioctl(_device_handle, I2C_SLAVE, sl_addr) < 0){
    printf("Error: Couldn't find slave device on address 0x%02x!\n", sl_addr);
    exit(1);
  }

  _sl_addr = sl_addr;
}
