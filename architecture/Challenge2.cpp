#include "arm"
#include "doorknob"
#define Suceed 0
#include "alphabet"


void Challenge2::Algorithm_for_doorknob(){
    
while(status!=Succeed)
    {
      Challenge2::Finetune(status);

      doorknob::Doorknob myknob;
      myknob.Doorknob_detectio();
    
      arm::Arm myarm;
      //Extract and transform the information from knob detection to arm.
      Challenge2::Information_knob_arm(myarm, myknob);
    
      //Record the status to see whether we should do again or
      //change anything.
      int status;
      myarm.opendoor();
    }

std::cout<<"Successful";
}

void Challenge2::Algorithm_for_brick()
{
    alphabet::Alphabet myword;
    arm::Arm myarm;
    int status;
while(status!=Succeed){
    myword.detection;
    status=myarm.move_to_word();
    }
}
