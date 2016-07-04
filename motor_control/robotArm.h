#include <VarSpeedServo.h>
// s0: shoulder(yaw)
// s1: shoulder(pitch)
// s2: elbow
// s3: wrist(pitch)
// s4: wrist(yaw)
// s5: gripper
struct ServoInfo {
    int min, max;   // hardware limits, should be within range
};

class robotArm {
  public:
    //Full constructor uses calibration data, or can just give pins
    robotArm();
    //required before running
    void begin(int pin_s0, int pin_s1, int pin_s2, int pin_s3, int pin_s4, int pin_s5);  
    void reset();
    //Travel smoothly from current point to another point
    void gotoPoint(float x, float y, float z);
    
    //Set servos to reach a certain point directly without caring how we get there 
    void goDirectlyTo(float x, float y, float z);

    void polarToCartesian(float theta, float r, float& x, float& y);
    //Same as above but for cylindrical polar coodrinates
    void gotoPointCylinder(float theta, float r, float z);
    void goDirectlyToCylinder(float theta, float r, float z);

    //Grab something
    void openGripper();
    //Let go of something
    void closeGripper();
    //Check to see if possible
    bool isReachable(float x, float y, float z);
    //Current x, y and z
    float getX();
    float getY();
    float getZ();

    float getR();
    float getTheta();
  private:
    //void polarToCartesian(float theta, float r, float& x, float& y);
    float _x, _y, _z;
    float _r, _t;
    VarSpeedServo s0, s1, s2, s3, s4, s5;
    ServoInfo s0_info, s1_info, s2_info, s3_info, s4_info, s5_info ;
    int pin_s0, pin_s1, pin_s2, pin_s3, pin_s4, pin_s5;
};