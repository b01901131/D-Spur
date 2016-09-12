#include "I2CBusController.h"
#include <fcntl.h>          // open(), close()
#include <linux/i2c-dev.h>  // i2c_smbus_write_byte()
#include <stdio.h>          // prinf()
#include <stdlib.h>         // exit()
#include <sys/ioctl.h>      // iotcl()


I2CBusController::I2CBusController(char* filename){
  if ((_device_handle = open(filename, O_RDWR)) < 0){
    perror("Error: Couldn't open device %d\n", _device_handle);
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

// private functions
void I2CBusController::setSlave(uint8_t sl_addr){
  if (ioctl(_device_handle, I2C_SLAVE, sl_addr) < 0){
    printf("Error: Couldn't find slave device on address 0x%02x!\n", sl_addr);
    exit(1);
  }

  _sl_addr = sl_addr;
}
