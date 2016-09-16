#ifndef WHEEL_CONTROLLER_H
#define WHEEL_CONTROLLER_H

#include <list>
#include <inttypes.h>

#include "I2CBusController.h"
#include "definitions.h"
#include "Gear.h"
#include "SerialPortController.h"

using namespace std;

// Coordinates are relative to robot, not world.
class WheelController {
  public:
    WheelController(I2CBusController* i2c_bus_controller,
                    uint8_t sl_addr);
    WheelController(SerialPortController* serial_port_controller);
    //~WheelController();

    void setMotionRect(double vx, double vy, double w0);
    //void setMotionPolar(double v, double theta, double w0);
    //void setVelocityX(double vx);
    //void setVelocityY(double vy);
    //void setAngVelocity(double w0);
    void stopMotion();
    /*
    vector<double> getMotionRect();
    vector<double> getMotionPolar();
    double getVelocityX();
    double getVelocityY();
    double getAngVelocity();
    */
  private:
    static WheelController* _me;

    void findGears(double target_w[4]);
    void findReverse(int wheel_idx, double target_w);
    int findAcceleration(int wheel_idx, double target_w, int shift_time);
    int findDeceleration(int wheel_idx, double target_w, int shift_time);
    bool existsInShiftQueue(int shift_time, list<Gear>::iterator& it);
    static void alarmHandler(int signum){
        _me->shiftGear();
    }
    void shiftGear();
    void sendCmd(Gear& g);
    
    bool _done[4];
    int _current_shift_time;
    int _current_gear_mode[4];
    double _current_w[4];

    list<Gear> _shift_queue;
    uint8_t _sl_addr;
    I2CBusController* _i2c_bus_controller;
    SerialPortController* _serial_port_controller;
};

#endif