#ifndef GEAR_H
#define GEAR_H

#include <inttypes.h>   // uint8_t
#include "definitions.h"

class Gear {
  friend class WheelController;
  
  public:
    Gear(int shift_time_us);
    //, double w1, int gear1, double w2, int gear2, double w3, int gear3, double w4, int gear4);
    void genCmd();
  
    int _shift_time_us;
    int _gear_mode[4];
    double _w[4];
    uint8_t _wheel_cmd[CMD_LEN];
};

#endif
