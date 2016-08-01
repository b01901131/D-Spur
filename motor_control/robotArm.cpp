#include "robotArm.h"
#include "kinematics.h"
#include <VarSpeedServo.h>
#include "Arduino.h"

// s0: shoulder(yaw)   24 90 156
// s1: shoulder(pitch) 23 85 153
// s2: elbow(pitch)    27 93 155
// s3: elbot(yaw)      25 90 155
// s4: wrist(pitch)    25 90 110
// s5: wrist(yaw)      25 90 152
// s6: gripper         25     50
//23 -75 -10 -44 -88 29

void setup_servo (ServoInfo& svo, const int n_min, const int n_init, const int n_max)
{
    svo.min = n_min;
    svo.max = n_max;
    svo.init  = n_init;
    svo.scale = 180. / (n_max - n_min);
}

void cmdGo(VarSpeedServo& s, ServoInfo& svo, int cmd, int speed, bool wait){
  int max = svo.max;
  int min = svo.min;
  cmd = svo.init+int(float(cmd)/svo.scale);
  //cmd += 90;
  if(cmd > max)
    cmd = max;
  else if(cmd < min)
    cmd = min;

  s.write(cmd, cmd, wait);
}

int angle2cmd (const ServoInfo& svo, const float angle, const int curr_cmd)
{
    float new_cmd = curr_cmd + angle;
    return int(new_cmd);
}

robotArm::robotArm() {
  setup_servo(s0_info, 22, 92, 157);
  setup_servo(s1_info, 23, 85, 157);
  setup_servo(s2_info, 25, 94, 155);
  setup_servo(s3_info, 25, 90, 155); 
  setup_servo(s4_info, 23, 87, 157);
  setup_servo(s5_info, 23, 90, 152); //gripper constraint
  setup_servo(s6_info, 25, 25,  50);
}

void robotArm::begin(int pin_s0, int pin_s1, int pin_s2, int pin_s3, int pin_s4, int pin_s5, int pin_s6) {
  s0.attach(pin_s0);
  s1.attach(pin_s1);
  s2.attach(pin_s2);
  s3.attach(pin_s3);
  s4.attach(pin_s4);
  s5.attach(pin_s5);
  s6.attach(pin_s6);
  reset();
}

void robotArm::reset(){
  delay(1000);
  // 25 - 90 - 155
  s0.write(92,127,true);
  // 23 - 85 - 153
  s1.write(85,127,false);
  // 27 - 93 - 155
  s2.write(94,127,false);
  // 25 - 90 - 155
  s3.write(90,127,false); 
  // 25 - 90 - 110 (metal will stuck)
  s4.write(87,127,false);
  // 23 - 90 - 152
  s5.write(90,127,false);  
  // 25 - 50
  s6.write(50,127,false); 
}

void robotArm::goTo(int cmd[7]){
  cmdGo(s0, s0_info, cmd[0], 127, true); // cmd[0]+90, 127, true);
  cmdGo(s1, s1_info, -cmd[1], 127, true); //-cmd[1]+88, 127, true);
  cmdGo(s2, s2_info, cmd[2], 127, true); // cmd[2]+93, 127, true);
  cmdGo(s3, s3_info, cmd[3], 127, true); // cmd[3]+90, 127, true);
  cmdGo(s4, s4_info, cmd[4], 127, true); // cmd[4]+90, 127, true);
  cmdGo(s5, s5_info, cmd[5], 127, true); // cmd[5]+90, 127, true);
}

//Travel smoothly from current point to another point
//void robotArm::gotoPoint(float x, float y, float z) {
//  //Starting points - current pos
//  float x0 = _x; 
//  float y0 = _y; 
//  float z0 = _z;
//  float dist = sqrt((x0-x)*(x0-x)+(y0-y)*(y0-y)+(z0-z)*(z0-z));
//  int step = 10;
//  for (int i = 0; i<dist; i+= step) {
//    goDirectlyTo(x0 + (x-x0)*i/dist, y0 + (y-y0) * i/dist, z0 + (z-z0) * i/dist);
//    delay(50);
//  }
//  goDirectlyTo(x, y, z);
//  delay(50);
//}
//
////Get x and y from theta and r
//void robotArm::polarToCartesian(float theta, float r, float& x, float& y){
//    _r = r;
//    _t = theta;
//    x = r*sin(theta);
//    y = r*cos(theta);
//}
//
////Same as above but for cylindrical polar coodrinates
//void robotArm::gotoPointCylinder(float theta, float r, float z){
//    float x, y;
//    polarToCartesian(theta, r, x, y);
//    gotoPoint(x,y,z);
//}
//
//void robotArm::goDirectlyToCylinder(float theta, float r, float z){
//    float x, y;
//    polarToCartesian(theta, r, x, y);
//    goDirectlyTo(x,y,z);
//}

//Check to see if possible
bool robotArm::isReachable(float x, float y, float z) {
  float cmd_0, cmd_1, cmd_2, cmd_3, cmd_4;
  return (solve(x, y, z, cmd_0, cmd_1, cmd_2, cmd_3, cmd_4)); 
}

//Grab something
void robotArm::openGripper() {
  s6.write(25, 90, true);
  //delay(300);
}

//Let go of something
void robotArm::closeGripper() {
  s6.write(50, 90, true);
  //delay(300);
}

//Current x, y and z
float robotArm::getX() {
  return _x;
}
float robotArm::getY() {
  return _y;
}
float robotArm::getZ() {
  return _z;
}


float robotArm::getR() {
  return _r;
}
float robotArm::getTheta() {
  return _t;
}