#include "robotArm.h"
#include "kinematics.h"
#include <VarSpeedServo.h>
#include "Arduino.h"

// s0: shoulder(yaw)
// s1: shoulder(pitch)
// s2: elbow
// s3: wrist(pitch)
// s4: wrist(yaw)
// s5: gripper
void setup_servo (ServoInfo& svo, const int n_min, const int n_max)
{
    svo.min = n_min;
    svo.max = n_max;
}

void cmdGo(VarSpeedServo& s, ServoInfo& svo, int cmd, int speed, bool wait){
  int max = svo.max;
  int min = svo.min;
  if(cmd > max)
    cmd = max;
  else if(cmd < min)
    cmd = min;
  s.write(cmd, speed, wait);
}

int angle2cmd (const ServoInfo& svo, const float angle, const int curr_cmd)
{
    float new_cmd = curr_cmd + angle;
    return int(new_cmd);
}

robotArm::robotArm() {
  setup_servo(s0_info, 25, 155);
  setup_servo(s1_info, 23, 153);
  setup_servo(s2_info, 27, 155);
  setup_servo(s3_info, 25, 155); 
  setup_servo(s4_info, 25, 110);
  setup_servo(s5_info, 23, 152); //gripper constraint
  setup_servo(s6_info,60, 105);
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
  //goDirectlyTo(0, 100, 50);
  //openGripper();
}

void robotArm::reset(){
  delay(1000);
  // 25 - 90 - 155
  s0.write(90,127,true);
  // 23 - 85 - 153
  s1.write(85,127,true);
  // 27 - 93 - 155
  s2.write(93,127,true);
  // 25 - 90 - 155
  s3.write(90,127,true); 
  // 25 - 90 - 110 (metal will stuck)
  s4.write(90,127,true);
  // 23 - 90 - 152
  s5.write(90,127,false);  
  // 25 - 50
  s6.write(50,127,false); // not tested yet (gripper)
}

void robotArm::goTo(int cmd[7]){
  cmdGo(s0, s0_info, cmd[0]+90, 127, true);
  cmdGo(s1, s1_info,-cmd[1]+88, 127, true);
  cmdGo(s2, s2_info, cmd[2]+93, 127, true);
  cmdGo(s3, s3_info, cmd[3]+90, 127, true);
  cmdGo(s4, s4_info, cmd[4]+90, 127, true);
  cmdGo(s5, s5_info, cmd[5]+90, 127, true);
  //s0.write(cmd[0]+90, 127, true);
  //s1.write(-cmd[1]+88, 127, true);
  //s2.write(cmd[2]+93, 127, true);
  //s3.write(cmd[3]+90, 127, true);
  //s4.write(cmd[4]+90, 127, true);
  //s5.write(cmd[5]+90, 127, true);
}
//Set servos to reach a certain point directly without caring how we get there 
void robotArm::goDirectlyTo(float x, float y, float z) {
  float cmd_0, cmd_1, cmd_2, cmd_3, cmd_4;
    
  //if (solve(x, y, z, cmd_0, cmd_1, cmd_2, cmd_3, cmd_4)) {
  //  s0.write(angle2cmd(s0_info, cmd_0, s0.read()));
  //  s1.write(angle2cmd(s1_info, cmd_1, s1.read()));
  //  s2.write(angle2cmd(s2_info, cmd_2, s2.read()));
  //  s3.write(angle2cmd(s3_info, cmd_3, s3.read()));
  //  s4.write(angle2cmd(s4_info, cmd_4, s4.read()));
  //  _x = x; _y = y; _z = z;
  //}    
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
  s5.write(25, 90, true);
  //delay(300);
}

//Let go of something
void robotArm::closeGripper() {
  s5.write(50, 90, true);
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