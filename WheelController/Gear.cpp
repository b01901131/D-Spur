#include "Gear.h"
#include <stdio.h> 

Gear::Gear(int shift_time_us)
{
  _shift_time_us = shift_time_us;
  _gear_mode[0] = 16;
  _gear_mode[1] = 16;
  _gear_mode[2] = 16;
  _gear_mode[3] = 16;
  _w[0] = 0;
  _w[1] = 0;
  _w[2] = 0;
  _w[3] = 0;
}

void Gear::genCmd()
{
  int wait_us[4];

  wait_us[0] = (_w[0] != 0)? (int) (PI * 10000 / (_gear_mode[0] * _w[0])) : -1;
  wait_us[1] = (_w[1] != 0)? (int) (PI * 10000 / (_gear_mode[1] * _w[1])) : -1;
  wait_us[2] = (_w[2] != 0)? (int) (PI * 10000 / (_gear_mode[2] * _w[2])) : -1;
  wait_us[3] = (_w[3] != 0)? (int) (PI * 10000 / (_gear_mode[3] * _w[3])) : -1;

  printf("wait_us[4] : %i, %i, %i, %i\n", wait_us[0],wait_us[1],wait_us[2],wait_us[3]);
  printf("_w[4] : %lf, %lf, %lf, %lf\n", _w[0],_w[1],_w[2],_w[3]);
  printf("_gear_mode[4] : %i, %i, %i, %i\n", _gear_mode[0],_gear_mode[1],_gear_mode[2],_gear_mode[3]);
  
  uint8_t dir_upper = 0;
  uint8_t dir_lower = 0;

  // determine step mode.
  switch(_gear_mode[0]){
    case 1:
      dir_upper |= (FULL << 4);
      break;
    case 2:
      dir_upper |= (HALF << 4);
      break;
    case 4:
      dir_upper |= (QUARTER << 4);
      break;
    case 8:
      dir_upper |= (EIGHTH << 4);
      break;
    case 16:
      dir_upper |= (SIXTEENTH << 4);
      break;
  }
  switch(_gear_mode[1]){
    case 1:
      dir_upper |= FULL;
      break;
    case 2:
      dir_upper |= HALF;
      break;
    case 4:
      dir_upper |= QUARTER;
      break;
    case 8:
      dir_upper |= EIGHTH;
      break;
    case 16:
      dir_upper |= SIXTEENTH;
      break;
  }
  switch(_gear_mode[2]){
    case 1:
      dir_lower |= (FULL << 4);
      break;
    case 2:
      dir_lower |= (HALF << 4);
      break;
    case 4:
      dir_lower |= (QUARTER << 4);
      break;
    case 8:
      dir_lower |= (EIGHTH << 4);
      break;
    case 16:
      dir_lower |= (SIXTEENTH << 4);
      break;
  }
  switch(_gear_mode[3]){
    case 1:
      dir_lower |= FULL;
      break;
    case 2:
      dir_lower |= HALF;
      break;
    case 4:
      dir_lower |= QUARTER;
      break;
    case 8:
      dir_lower |= EIGHTH;
      break;
    case 16:
      dir_lower |= SIXTEENTH;
      break;
  }
  
  // determine direction.
  if(wait_us[0] < 0)
    dir_upper += (1 << 7);    
  if(wait_us[1] > 0)
    dir_upper += (1 << 3);
  if(wait_us[2] < 0)
    dir_lower += (1 << 7);
  if(wait_us[3] > 0)
    dir_lower += (1 << 3);
  printf("dir_upper : 0x%02x\ndir_lower : 0x%02x\n", dir_upper, dir_lower);

  // abs value.
  wait_us[0] = (wait_us[0] > 0)? wait_us[0]: -wait_us[0];
  wait_us[1] = (wait_us[1] > 0)? wait_us[1]: -wait_us[1];
  wait_us[2] = (wait_us[2] > 0)? wait_us[2]: -wait_us[2];
  wait_us[3] = (wait_us[3] > 0)? wait_us[3]: -wait_us[3];

  for (int i = 0; i< 4; i++){
    _wheel_cmd[0+4*i] = (wait_us[i] >> 24) & 0xff;
    _wheel_cmd[1+4*i] = (wait_us[i] >> 16) & 0xff;
    _wheel_cmd[2+4*i] = (wait_us[i] >>  8) & 0xff;
    _wheel_cmd[3+4*i] = (wait_us[i] & 0xff);
  }
  _wheel_cmd[16] = dir_upper;
  _wheel_cmd[17] = dir_lower;
}
