#include "Control_System"

void Challenge1::Algorithm(){
  while(1){
    Control_System::Obstacle_Avoidance();
    Control_System::Detect_and_Pass_Seesaw();
    Control_System::React_for_Sign();
    Control_Sytem::Lane_Following();
    if(Control_System::Challenge1_End()) break;
  }
}

