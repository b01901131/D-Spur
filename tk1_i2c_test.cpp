#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

int main (void){
    int device_handle;
    if ((device_handle = open("/dev/i2c-1", O_RDWR)) < 0){
        printf("Error: Couldn't open device %d\n", device_handle);
        return 1;
    }

    if (ioctl(device_handle, I2C_SLAVE, 4) < 0){
        printf("Error: Couldn't find arduino on address!\n");
        return 1;
    }

    char data[4] = {0x65, 0x66, 0x67, 0x68};
    i2c_smbus_write_byte(device_handle, data[0]);
    i2c_smbus_write_byte(device_handle, data[1]);
    i2c_smbus_write_byte(device_handle, data[2]);
    i2c_smbus_write_byte(device_handle, data[3]);
    //write(device_handle, data, 4);
    char b = i2c_smbus_read_byte(device_handle);
    printf("%i\n",b);

    close(device_handle);

    return 0;
}
