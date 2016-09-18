#ifndef SERIAL_PORT_CONTROLLER_H
#define SERIAL_PORT_CONTROLLER_H

#include <inttypes.h>
#include "definitions.h"

class SerialPortController {
  public:
    SerialPortController(char* filename);
    ~SerialPortController();

    //void sendByte(uint8_t data);
    void sendBytes(uint8_t* data, int len);

  private:

    int _device_handle;
};


#endif
