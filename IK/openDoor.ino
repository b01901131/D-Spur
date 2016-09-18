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
int num_of_cmd = 9;
int num_of_motor = 6;
double** servo_cmd = new double*[num_of_cmd];

void reset(){
  index = 0;
  row = 0;
  col = 0;
  state = 0;
}

void calcIK(){
  if(Serial.available() > 0){
    char inChar = Serial.read();
    if(inChar=='r'){
      reset();
      arm.reset();
    }
    if(inChar=='c'){
      reset();
      arm.cameraMode();
    }
    else{
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
        row = index/num_of_motor;
        col = index%num_of_motor;
        servo_cmd[row][col] = atof(cmd.c_str());        
        //Serial.println(row);
        //Serial.println(col);
        //Serial.println(servo_cmd[row][col]);
        cmd = "";
        index++;
        if(index==num_of_cmd*num_of_motor)
          state = 2;
      }
    }
    } 
  }
}

void setup(){
  Serial.begin(9600);
  arm.begin(2,3,4,5,6,7,8);
  for(int i = 0; i < num_of_cmd; ++i)
    servo_cmd[i] = new double[num_of_motor];
  arm.shootPos();
}

void loop(){  
  calcIK();
  if(state==2){
    reset();
    for(int i=0; i<num_of_cmd; i++)
      for(int j=0; j < num_of_motor; j++)
        Serial.println(servo_cmd[i][j]);
    
    arm.openGripper();
    arm.goTo(servo_cmd[0]);
    //delay(500);
    //arm.closeGripper();
    //
    //for(int i=1; i<num_of_cmd; i++){
    //  Serial.println(i);
    //  //arm.goTo(servo_cmd[i]);
    //  delay(100);
    //}
//
    //delay(500);
    //arm.openGripper();
    //arm.reset();
  }
 
  
}
