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

int angle2cmd (const ServoInfo& svo, const float angle, const int curr_cmd)
{
    float new_cmd = curr_cmd + angle;
    return int(new_cmd);
}

robotArm::robotArm() {
  setup_servo(s0_info, 0, 180);
  setup_servo(s1_info, 0, 180);
  setup_servo(s2_info, 0, 180);
  setup_servo(s3_info, 0, 180); //watch out this one(not sure whether it will stuck or not)
  setup_servo(s4_info, 0, 180);
  setup_servo(s5_info,60, 105); //gripper constraint

}

void robotArm::begin(int pin_s0, int pin_s1, int pin_s2, int pin_s3, int pin_s4, int pin_s5) {
  s0.attach(pin_s0);
  s1.attach(pin_s1);
  s2.attach(pin_s2);
  s3.attach(pin_s3);
  s4.attach(pin_s4);
  s5.attach(pin_s5);
  reset();
  //goDirectlyTo(0, 100, 50);
  //openGripper();
}

void robotArm::reset(){
  delay(1000);
  s0.write(90,127,true);
  s1.write(88,127,true);
  s2.write(82,127,true);
  s3.write(127,127,true); 
  
  s4.write(0,127,false);  // not tested yet (wrist)
  s5.write(60,127,false); // not tested yet (gripper)
  //delay(1000);
  //s0.write(90,255,true);
  //delay(1000);
  //s3.write(40,30,true);
  //delay(1000);
  //s2.write(100,45,true);
  //delay(1000);
  //s0.write(30,127,false);
  //s2.write(82,127,false);
  //s3.write(127,127,true);
}
//Set servos to reach a certain point directly without caring how we get there 
void robotArm::goDirectlyTo(float x, float y, float z) {
  float cmd_0, cmd_1, cmd_2, cmd_3, cmd_4;
  if (solve(x, y, z, cmd_0, cmd_1, cmd_2, cmd_3, cmd_4)) {
    s0.write(angle2cmd(s0_info, cmd_0, s0.read()));
    s1.write(angle2cmd(s1_info, cmd_1, s1.read()));
    s2.write(angle2cmd(s2_info, cmd_2, s2.read()));
    s3.write(angle2cmd(s3_info, cmd_3, s3.read()));
    s4.write(angle2cmd(s4_info, cmd_4, s4.read()));
    _x = x; _y = y; _z = z;
  }    
}

//Travel smoothly from current point to another point
void robotArm::gotoPoint(float x, float y, float z) {
  //Starting points - current pos
  float x0 = _x; 
  float y0 = _y; 
  float z0 = _z;
  float dist = sqrt((x0-x)*(x0-x)+(y0-y)*(y0-y)+(z0-z)*(z0-z));
  int step = 10;
  for (int i = 0; i<dist; i+= step) {
    goDirectlyTo(x0 + (x-x0)*i/dist, y0 + (y-y0) * i/dist, z0 + (z-z0) * i/dist);
    delay(50);
  }
  goDirectlyTo(x, y, z);
  delay(50);
}

//Get x and y from theta and r
void robotArm::polarToCartesian(float theta, float r, float& x, float& y){
    _r = r;
    _t = theta;
    x = r*sin(theta);
    y = r*cos(theta);
}

//Same as above but for cylindrical polar coodrinates
void robotArm::gotoPointCylinder(float theta, float r, float z){
    float x, y;
    polarToCartesian(theta, r, x, y);
    gotoPoint(x,y,z);
}

void robotArm::goDirectlyToCylinder(float theta, float r, float z){
    float x, y;
    polarToCartesian(theta, r, x, y);
    goDirectlyTo(x,y,z);
}

//Check to see if possible
bool robotArm::isReachable(float x, float y, float z) {
  float cmd_0, cmd_1, cmd_2, cmd_3, cmd_4;
  return (solve(x, y, z, cmd_0, cmd_1, cmd_2, cmd_3, cmd_4)); 
}

//Grab something
void robotArm::openGripper() {
  s5.write(60, 90, true);
  //delay(300);
}

//Let go of something
void robotArm::closeGripper() {
  s5.write(105, 90, true);
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