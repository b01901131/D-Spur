#include <VarSpeedServo.h>
#include <robotArm.h>
#include <String.h>

using namespace std;
//VarSpeedServo s0;
String cmd;
int i = 0;
int servo_cmd_1[7] = {0,0,0,0,0,0,0};
int servo_cmd_2[7] = {0,0,0,0,0,0,0};
robotArm arm;
int sent_state = 1; //1:cmd_1, 2:cmd_2, 0:finish

void calcIK(){
  if(Serial.available() > 0){
    char curr = Serial.read();
    if(curr == ' ' && sent_state == 1){
      servo_cmd_1[i] = cmd.toInt();
      cmd = "";
      i ++;
      if(i == 6){
        sent_state = 2;
        i = 0;
      }
    }
    else if(curr == ' ' && sent_state == 2){
      servo_cmd_2[i] = cmd.toInt();
      cmd = "";
      i ++;
      if(i == 6){
        sent_state = 0;
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
  if(sent_state == 0){
    for(int j=0; j < 7; j++)
      //Serial.println(servo_cmd[j]);
    sent_state = 1;
    Serial.println(float(servo_cmd_1[0]));
    Serial.println(-float(servo_cmd_1[1]));
    Serial.println(float(servo_cmd_1[2]));
    Serial.println(float(servo_cmd_1[3]));
    Serial.println(float(servo_cmd_1[4]));
    Serial.println(float(servo_cmd_1[5]));
    Serial.println(float(servo_cmd_2[0]));
    Serial.println(-float(servo_cmd_2[1]));
    Serial.println(float(servo_cmd_2[2]));
    Serial.println(float(servo_cmd_2[3]));
    Serial.println(float(servo_cmd_2[4]));
    Serial.println(float(servo_cmd_2[5]));
    arm.openGripper();
    
    arm.goTo(servo_cmd_1);
    Serial.println("first position");
    delay(2000);
    
    arm.goTo(servo_cmd_2);
    Serial.println("second position");
    delay(1000);
    
    arm.closeGripper();
    Serial.println("close gripper");
    delay(1000);
    
    arm.goTo(servo_cmd_1);
    Serial.println("reset");
    arm.reset(); 
  }
 
  
}

