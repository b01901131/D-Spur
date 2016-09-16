#include <iostream>     // cout()
#include <unistd.h>
#include <stdio.h>      // printf(), sprintf()
#include <string.h>
#include <inttypes.h>   // uint8_t
//#include "SerialPortController.h"
#include "LcdController.h"
#include "WheelController.h"

#define PI 3.1415926

using namespace std;

int main (void){
  I2CBusController* i2c_bus = new I2CBusController("/dev/i2c-0");

  LcdController lcd(i2c_bus, 0x27, 16, 2);
  lcd.init();
  char upper_msg[16];
  char lower_msg[16];
  int n;
  n = sprintf (upper_msg, "Hello, world!");
  lcd.print(upper_msg, n);

  WheelController* wheel = new WheelController(i2c_bus, 0x4);

  char cmd[20];
  double vx = 0;
  double vy = 0;
  double w0 = 0;

  while (1) {
    scanf("%s %lf %lf %lf", cmd, &vx, &vy, &w0);
    if (cmd[0] == 'q')
      break;
    else if(cmd[0] == 's'){
      wheel->stopMotion();
      n = sprintf (upper_msg, "vx=%.2lf, vy=%.2lf", 0.0, 0.0);
      lcd.setCursor(0,0);
      lcd.print(upper_msg, n);
      lcd.setCursor(0,1);
      n = sprintf (lower_msg, "w0=%.3lf", 0.0);
      lcd.print(lower_msg, n);
    } else {
      printf("main: vx=%lf, vy=%lf, w0=%lf\n", vx, vy, w0);

      n = sprintf (upper_msg, "vx=%.2lf, vy=%.2lf", vx, vy);
      lcd.setCursor(0,0);
      lcd.print(upper_msg, n);
      lcd.setCursor(0,1);
      n = sprintf (lower_msg, "w0=%.3lf", w0);
      lcd.print(lower_msg, n);

      wheel->setMotionRect(vx, vy, w0);
    
      memset(upper_msg,0,strlen(upper_msg));
      memset(lower_msg,0,strlen(lower_msg));
    }
  }

  return 0;
}
