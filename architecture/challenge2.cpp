#include "image_detection"
#include "wheel_control"

//Need to check the model.
//Do I need a while loop here?
//Right now, my model is that, it will issue a command when the switch is on.

void Control_System::Lane_Following(bool switch)
{
 if(switch){
    slope=Slope_Filter();
    Wheel_Control(slope);
    }
 else{
    //It will issue a command that stop the wheel.
    Wheel_Control_Stop();
 }
}

#define Red_Sign 2;
#define Green_Sign 3;


void Control_Sytstem::React_For_Sign()
{
  int sign;
  sign=Sign_Detection();
  if(sign == Red_Sign){
    Wheel_Control_Stop();
  }
  else if (sign ==Green_Sign){
  //We don't need slope detection everytime. 
  //So , it might be good to have a function to use the old slope.
  //Moreover, if the latest detection failed, it should have some way
  //to resolve.
  Wheel_Control();
  
  }
}

void Control_System::Obstacle_Avoidance(){
  
}


