#include <VarSpeedServo.h>
#include <robotArm.h>
#include <String.h>

using namespace std;
//VarSpeedServo s0;
String cmd;
int i = 0;
int servo_cmd[7] = {0,0,0,0,0,0,0};
robotArm arm;
bool sent_fl = false;

void calcIK(){
  if(Serial.available() > 0){
    char curr = Serial.read();
    if(curr == ' ' && sent_fl == false){
      servo_cmd[i] = cmd.toInt();
      cmd = "";
      i ++;
      if(i == 6){
        sent_fl = true;
        i = 0;
      }
    }
    cmd += curr;
  }
}

void setup(){
  Serial.begin(9600);
  arm.begin(2,3,4,5,6,7,8);
}

void loop(){
  calcIK();
  if(sent_fl){
    for(int j=0; j < 7; j++)
      //Serial.println(servo_cmd[j]);
    sent_fl = false;
    Serial.println(servo_cmd[0]+90);
    Serial.println(-servo_cmd[1]+88);
    Serial.println(servo_cmd[2]+93);
    Serial.println(servo_cmd[3]+90);
    Serial.println(servo_cmd[4]+90);
    Serial.println(servo_cmd[5]+90);
    arm.goTo(servo_cmd);
  }
 
  
}

