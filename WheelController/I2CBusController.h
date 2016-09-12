#ifndef I2C_BUS_CONTROLLER_H
#define I2C_BUS_CONTROLLER_H

class I2CBusController {
  public:
    I2CBusController(char* filename);
    ~I2CBusController();

    void sendByte(uint8_t sl_addr, uint8_t data);

  private:
    void setSlave(uint8_t sl_addr);

    int _device_handle;
    uint8_t _sl_addr;
};


#endif