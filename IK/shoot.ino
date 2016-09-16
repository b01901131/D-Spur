#include <VarSpeedServo.h>
#include <robotArm.h>
#include <String.h>

using namespace std;

String cmd;
robotArm arm;
int state = 0;
int index = 0;
int row = 0;
int col = 0;
int num_of_cmd = 1;
int** servo_cmd = new int*[num_of_cmd];

void calcIK(){
  if(Serial.available() > 0){
    char inChar = Serial.read();
    if(state==0){
      if(num_of_cmd != (int)inChar-48){
        Serial.println("Invalid number of cmd");
        state = 2;
      }
      else{
        Serial.println("Command received");
        state = 1;
      }
    }
    else if(state==1){
      cmd += inChar;
      if(inChar==' '){
        row = index/3;
        col = index%3;
        servo_cmd[row][col] = cmd.toInt();        
        cmd = "";
        index++;
        if(index==num_of_cmd*3)
          state = 2;
      }
    }
    
  }
}

void setup(){
  Serial.begin(9600);
  arm.begin(2,3,4,5,6,7,8);
  for(int i = 0; i < num_of_cmd; ++i)
    servo_cmd[i] = new int[3];
}

void reset(){
  index = 0;
  row = 0;
  col = 0;
  state = 0;
}
void loop(){  
  calcIK();
  if(state==2){
    reset();
    arm.shootPos();
    for(int i=0; i<num_of_cmd; i++)
      for(int j=0; j < 3; j++)
        Serial.println(servo_cmd[i][j]);
    
    //arm.openGripper();
    //arm.goTo(servo_cmd[0]);
    //delay(500);
    //arm.closeGripper();
    //
    //for(int i=1; i<num_of_cmd; i++){
    //  Serial.println(i);
    //  arm.goTo(servo_cmd[i]);
    //  delay(100);
    //}

    delay(500);
    //arm.openGripper();
    arm.shootPos();
  }
 
  
}
