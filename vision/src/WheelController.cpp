#include "WheelController.h"
#include <cmath>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

//using namespace std;

WheelController* WheelController::_me;


WheelController::WheelController( I2CBusController* i2c_bus_controller,
                                  uint8_t sl_addr)
{
  _i2c_bus_controller = i2c_bus_controller;
  _sl_addr = sl_addr;
  _me = this;

  _current_shift_time = 0;
  _current_gear_mode[0] = 16;
  _current_gear_mode[1] = 16;
  _current_gear_mode[2] = 16;
  _current_gear_mode[3] = 16;
  _current_w[0] = 0;
  _current_w[1] = 0;
  _current_w[2] = 0;
  _current_w[3] = 0;
/*
  _done[0] = false;
  _done[1] = false;
  _done[2] = false;
  _done[3] = false;
*/  

  if(DEBUG)
    printf("Done initializing WheelController (i2c)\n");
}


WheelController::WheelController(SerialPortController* serial_port_controller)
{
  _serial_port_controller = serial_port_controller;
  _me = this;

  _current_shift_time = 0;
  _current_gear_mode[0] = 16;
  _current_gear_mode[1] = 16;
  _current_gear_mode[2] = 16;
  _current_gear_mode[3] = 16;
  _current_w[0] = 0;
  _current_w[1] = 0;
  _current_w[2] = 0;
  _current_w[3] = 0;
/*  
  _done[0] = false;
  _done[1] = false;
  _done[2] = false;
  _done[3] = false;
*/
  if(DEBUG)
    printf("Done initializing WheelController (serial)\n");
}

void WheelController::setMotionRect(double vx, double vy, double w0)
{
  /*
   *    transmformation matrix
   *
   *    | w1 |         3       | 1     1    -0.3475 |   | vy |
   *    | w2 | =  __________   | 1    -1     0.3475 | * | vx |
   *    | w3 |                 | 1    -1    -0.3475 |   | w0 |
   *    | w4 |      0.0635     | 1     1     0.3475 |
   *
   *    w = 1.8/16(step_mode) *PI/180 * 10^6 / wait_us
   *    wait_us = 1.8*10^6*PI / (16*180*w)
   */
  if(DEBUG)
    printf("setMotionRect\n");

  double target_w[4];

  target_w[0] = (vy +vx -0.3475 * w0) *3 / (0.0635);
  target_w[1] = (vy -vx +0.3475 * w0) *3 / (0.0635);
  target_w[2] = (vy -vx -0.3475 * w0) *3 / (0.0635);
  target_w[3] = (vy +vx +0.3475 * w0) *3 / (0.0635);

  if(DEBUG)
    printf("target_w[0] : %lf\ntarget_w[1] : %lf\ntarget_w[2] : %lf\ntarget_w[3] : %lf\n",
          target_w[0],target_w[1],target_w[2],target_w[3]);

  _current_shift_time = 0;
  findGears(target_w);

  if(DEBUG){
    for (list<Gear>::iterator it = _shift_queue.begin(); it != _shift_queue.end(); it++){
      printf("Gear : shift_time=%i, _w=%lf,%lf,%lf,%lf , _gear_mode=%i,%i,%i,%i\n",
       it->_shift_time_us, it->_w[0],it->_w[1],it->_w[2],it->_w[3], 
       it->_gear_mode[0],it->_gear_mode[1],it->_gear_mode[2],it->_gear_mode[3]);
    }
    printf("findGears() done.\n\n");
  }

  signal(SIGALRM, WheelController::alarmHandler);
  shiftGear();
}

/*
void setMotionPolar(double v, double theta, double w0)
{

}
void setVelocityX(double vx);
void setVelocityY(double vy);
void setAngVelocity(double w0);
*/

void WheelController::stopMotion()
{
  uint8_t wheel_cmd[CMD_LEN];
  short stop = -1;
  for(int i = 0; i < 4; i++){
    //wheel_cmd[0+4*i] = (stop >> 24) & 0xff;
    //wheel_cmd[1+4*i] = (stop >> 16) & 0xff;
    wheel_cmd[0+2*i] = (stop >> 8) & 0xff;
    wheel_cmd[1+2*i] = (stop & 0xff);
  }
  wheel_cmd[8] = 0x7f;
  wheel_cmd[9] = 0x7f;

  if(_serial_port_controller != NULL)
    _serial_port_controller->sendBytes(wheel_cmd, CMD_LEN);

  //if(_i2c_bus_controller != NULL)
  //  _i2c_bus_controller->sendBytes(_sl_addr, wheel_cmd, CMD_LEN);

  _current_gear_mode[0] = 16;
  _current_gear_mode[1] = 16;
  _current_gear_mode[2] = 16;
  _current_gear_mode[3] = 16;
  _current_w[0] = 0;
  _current_w[1] = 0;
  _current_w[2] = 0;
  _current_w[3] = 0;
/*
  _done[0] = false;
  _done[1] = false;
  _done[2] = false;
  _done[3] = false;
*/
}

// private functions
void WheelController::findGears(double target_w[4])
{
  int i;
  bool done[4] = {false, false, false, false};
  // for each wheel
  for(i = 0; i < 4; i++){
    if(_current_w[i] * target_w[i] < 0){
      // reverse direction.
      findReverse(i, target_w[i]);
    } else {
      if(abs(target_w[i]) > abs(_current_w[i])){
        //printf("findGears() : wheel %i acceleration.\n", i);
        findAcceleration(i, target_w[i], 0);
      } else if(target_w[i] == _current_w[i]){
        done[i] = true;
      } else {
        findDeceleration(i, target_w[i], 0);
      }
    }
  }

  // check if last gear not target.
  int new_shift_time = 3000;
  for(list<Gear>::iterator it = _shift_queue.begin(); it != _shift_queue.end(); it++){
    // find which wheel has reached target speed
    for(i = 0; i < 4 ; i++){
      if(it->_w[i] == target_w[i] && !done[i])
        done[i] = true;
    }

    for(i = 0; i < 4 ; i++){
      if(done[i] && it->_w[i] != target_w[i])
        it->_w[i] = target_w[i];
    }
    
    it->_shift_time_us = new_shift_time;
    new_shift_time += 3000;
  }
}

void WheelController::findReverse(int wheel_idx, double target_w)
{
  int shift_time = 0;
  if(_current_w[wheel_idx] > 0){
    if(DEBUG){
      printf("reverse %i : from positive to negative.\n", wheel_idx);
      printf("from %lf to %lf.\n", _current_w[wheel_idx], target_w);
    }
    shift_time = findDeceleration(wheel_idx, MIN_ANG_VELOCITY[4], shift_time);
    Gear reverse(shift_time);
    reverse._gear_mode[wheel_idx] = _current_gear_mode[wheel_idx];
    reverse._w[wheel_idx] = -1 * MIN_ANG_VELOCITY[4];
    if(DEBUG)
      printf("accelerate to target velocity.\n");
    findAcceleration(wheel_idx, target_w, shift_time);
  } else {
    // from negative to postive.
    shift_time = findDeceleration(wheel_idx, -1 * MIN_ANG_VELOCITY[4], shift_time);
    Gear reverse(shift_time);
    reverse._gear_mode[wheel_idx] = _current_gear_mode[wheel_idx];
    reverse._w[wheel_idx] = MIN_ANG_VELOCITY[4];
    findAcceleration(wheel_idx, target_w, shift_time);  
  }
}

int WheelController::findAcceleration(int wheel_idx, double target_w, int shift_time=0)
{
  list<Gear>::iterator it;

  int gear_idx = 0;
  int gear_mode = _current_gear_mode[wheel_idx];
  while (gear_mode != 1){
    gear_mode /= 2;
    gear_idx++;
  }
  gear_mode = _current_gear_mode[wheel_idx];
  double shift_speed;
  double start_speed = _current_w[wheel_idx];

  //gear_idx++;
  for (; gear_idx >= 0;){
    shift_speed = (target_w > 0)? MAX_ANG_VELOCITY[gear_idx] : -MAX_ANG_VELOCITY[gear_idx];

    if(DEBUG){
      printf("acceleration : gear_idx = %i\n", gear_idx);
      printf("acceleration : gear_mode = %i\n", gear_mode);
      printf("wheel %i : target_w=%lf, MAX_ANG_VELOCITY[gear_idx]=%lf\n", wheel_idx, target_w, MAX_ANG_VELOCITY[gear_idx]);
      printf("shift_speed:%lf, start_speed:%lf\n", shift_speed, start_speed);  
    }
    
    if (abs(target_w) <= MAX_ANG_VELOCITY[gear_idx]){
      // no need to shift up.
      //shift_time +=  ((abs(target_w) - abs(start_speed)) / ANGULAR_ACCELERATION) + 3000;
      shift_time += 3000;
      
      if( !existsInShiftQueue(shift_time, it) ){
        // add a gear if not exist.
        Gear to(shift_time);
        to._gear_mode[wheel_idx] = gear_mode;
        to._w[wheel_idx] = target_w;

        _shift_queue.insert(it, to);
      } else {
        // update gear if already exists.
        it->_gear_mode[wheel_idx] = gear_mode;
        it->_w[wheel_idx] = target_w;
      }

      // update _current_gear_mode & _current_w
      _current_w[wheel_idx] = target_w;
      _current_gear_mode[wheel_idx] = gear_mode;
      
      if(DEBUG)
        printf("end of acceleration\n");
      break;
    }
    // needs to shift up. first accelerate to shift speed (1000us) 
    //shift_time +=  ((abs(shift_speed) - abs(start_speed)) / ANGULAR_ACCELERATION) + 3000;
    shift_time += 3000;
     
    if( !existsInShiftQueue(shift_time, it) ){
      // add a gear if not exist.
      Gear from(shift_time);
      from._gear_mode[wheel_idx] = gear_mode;
      from._w[wheel_idx] = shift_speed;
      if(DEBUG)
        printf("Add : gear(%i, %lf)\n", gear_mode, shift_speed);
        
      Gear to(shift_time);
      to._gear_mode[wheel_idx] = gear_mode / 2;
      to._w[wheel_idx] = shift_speed;
      if(DEBUG)
        printf("Add : gear(%i, %lf)\n", gear_mode/2, shift_speed);

      _shift_queue.insert(it, from);
      _shift_queue.insert(it, to);
    } else {
      // update gear if already exists.
      it->_gear_mode[wheel_idx] = gear_mode;
      it->_w[wheel_idx] = shift_speed;
      if(DEBUG)
        printf("Update : gear(%i, %lf)\n", gear_mode, shift_speed);

      it++;
      it->_gear_mode[wheel_idx] = gear_mode / 2;
      it->_w[wheel_idx] = shift_speed;
      if(DEBUG)
        printf("Update : gear(%i, %lf)\n", gear_mode/2, shift_speed);
    }
    start_speed = shift_speed;
    gear_idx--;
    gear_mode = (1 << gear_idx);
  }

  return shift_time;
}
    
int WheelController::findDeceleration(int wheel_idx, double target_w, int shift_time=0)
{
  list<Gear>::iterator it;

  int gear_idx = 0;
  int gear_mode = _current_gear_mode[wheel_idx];
  while (gear_mode != 1){
    gear_mode /= 2;
    gear_idx++;
  }
  gear_mode = _current_gear_mode[wheel_idx];
  double shift_speed;
  double start_speed = _current_w[wheel_idx];

  for (; gear_idx < 5;){
    shift_speed = (target_w > 0)? MIN_ANG_VELOCITY[gear_idx] : -MIN_ANG_VELOCITY[gear_idx];
    if(DEBUG){
      printf("deceleration : gear_idx = %i\n", gear_idx);
      printf("deceleration : gear_mode = %i\n", gear_mode);
      printf("wheel %i : target_w=%lf, MIN_ANG_VELOCITY[gear_idx]=%lf\n", wheel_idx, target_w, MIN_ANG_VELOCITY[gear_idx]);
      printf("shift_speed:%lf, start_speed:%lf\n", shift_speed, start_speed);  
    }
    
    if (abs(target_w) >= MIN_ANG_VELOCITY[gear_idx]){
      // no need to shift down.
      //shift_time +=  ((abs(start_speed) - abs(target_w)) / ANGULAR_ACCELERATION) + 3000;
      shift_time += 3000;
      if( !existsInShiftQueue(shift_time, it) ){
        // add a gear if not exist.
        Gear to(shift_time);
        to._gear_mode[wheel_idx] = gear_mode;
        to._w[wheel_idx] = target_w;

        _shift_queue.insert(it, to);
      } else {
        // update gear if already exists.
        it->_gear_mode[wheel_idx] = gear_mode;
        it->_w[wheel_idx] = target_w;
      }

      // update _current_gear_mode & _current_w
      _current_w[wheel_idx] = target_w;
      _current_gear_mode[wheel_idx] = gear_mode;
      if(DEBUG)
        printf("end of deceleration\n");
      break;
    }

    // needs to shift down. first decelerate to start speed (2000us) 
    //shift_time +=  ((abs(start_speed) - abs(shift_speed)) / ANGULAR_ACCELERATION) + 3000;
    shift_time += 3000;
     
    if( !existsInShiftQueue(shift_time, it) ){
      // add a gear if not exist.
      Gear from(shift_time);
      from._gear_mode[wheel_idx] = gear_mode;
      from._w[wheel_idx] = shift_speed;
      if(DEBUG)
        printf("Add : gear(%i, %lf)\n", gear_mode, shift_speed);
        
      Gear to(shift_time);
      to._gear_mode[wheel_idx] = gear_mode * 2;
      to._w[wheel_idx] = shift_speed;
      if(DEBUG)
        printf("Add : gear(%i, %lf)\n", gear_mode*2, shift_speed);

      _shift_queue.insert(it, from);
      _shift_queue.insert(it, to);
    } else {
      // update gear if already exists.
      it->_gear_mode[wheel_idx] = gear_mode;
      it->_w[wheel_idx] = shift_speed;
      if(DEBUG)
        printf("Update : gear(%i, %lf)\n", gear_mode, shift_speed);

      it++;
      it->_gear_mode[wheel_idx] = gear_mode * 2;
      it->_w[wheel_idx] = shift_speed;
      if(DEBUG)
        printf("Update : gear(%i, %lf)\n", gear_mode*2, shift_speed);
    }
    start_speed = shift_speed;
    gear_idx++;
    gear_mode = (1 << gear_idx);
  }
  return shift_time;
}

bool WheelController::existsInShiftQueue(int shift_time, list<Gear>::iterator& it)
{
  //printf("shift queue size = %i\n", _shift_queue.size());
  bool exists = false;
  for (it = _shift_queue.begin(); it != _shift_queue.end(); it++){
    if( it->_shift_time_us >= shift_time ){
      if (it->_shift_time_us == shift_time)
        exists = true;
      break;
    }
  }

  return exists;
}

void WheelController::shiftGear()
{
  if(_shift_queue.size() == 0){
    if(DEBUG)
      printf("shiftGear done !!\n");

    signal(SIGALRM, SIG_DFL);
    return;
  }
  
  if(DEBUG)
    printf("shiftGear\n");

  Gear g = _shift_queue.front();
  int time_interval = g._shift_time_us - _current_shift_time;
  if(DEBUG){
    printf("time_interval=%i\n", time_interval);
    printf("Gear : shift_time=%i, _w=%lf,%lf,%lf,%lf , _gear_mode=%i,%i,%i,%i\n",
      g._shift_time_us, g._w[0],g._w[1],g._w[2],g._w[3], 
      g._gear_mode[0],g._gear_mode[1],g._gear_mode[2],g._gear_mode[3]);
    printf("_current_shift_time=%i\n", _current_shift_time);
  }
  
  g.genCmd();
  sendCmd(g);
  //signal(SIGALRM, WheelController::alarmHandler);
  _shift_queue.pop_front();

  // if there is shifting gear behind.
  if(_shift_queue.front()._shift_time_us == _current_shift_time + time_interval){
    g = _shift_queue.front();
    g.genCmd();
    sendCmd(g);
    _shift_queue.pop_front();
  }

  ualarm(time_interval, 0);
  _current_shift_time += time_interval;
  if(DEBUG)
    printf("_current_shift_time=%i\n", _current_shift_time);
}

void WheelController::sendCmd(Gear& g)
{
  if(DEBUG)
    printf("sendCmd\n\n");

  if(_serial_port_controller != NULL)
    _serial_port_controller->sendBytes(g._wheel_cmd, CMD_LEN);

  //if(_i2c_bus_controller != NULL)
  //  _i2c_bus_controller->sendBytes(_sl_addr, g._wheel_cmd, CMD_LEN);
}

