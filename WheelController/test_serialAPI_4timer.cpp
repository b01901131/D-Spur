#include <iostream>     // cout()
#include <unistd.h>
#include <stdio.h>      // printf()
#include <string>
#include <inttypes.h>   // uint8_t
#include "SerialPortController.h"
#include "WheelController.h"

#define PI 3.1415926

using namespace std;

int main (void){
  //SerialPortController* serial_port_controller = new SerialPortController("/dev/cu.wchusbserial1410");
  SerialPortController* serial_port_controller = NULL;

  WheelController* wheel = new WheelController(serial_port_controller);

  //uint8_t wheel_protocol[22];
  char cmd[20];
  double vx = 0;
  double vy = 0;
  double w0 = 0;

  //double w1,w2,w3,w4;
  //int wait_us1,wait_us2,wait_us3,wait_us4;
  //int step_mode = 16;

  while (1) {
    scanf("%s %lf %lf %lf", cmd, &vx, &vy, &w0);
    if (cmd[0] == 'q')
      break;
    else if(cmd[0] == 's'){
      /*
      int stop = -1;

      wheel_protocol[0] = (stop >> 24) & 0xff;
      wheel_protocol[1] = (stop >> 16) & 0xff;
      wheel_protocol[2] = (stop >> 8) & 0xff;
      wheel_protocol[3] = (stop & 0xff);

      serial_port_controller->sendByte(wheel_protocol, 18);
      */
      wheel->stopMotion();
      continue;
    }
    printf("main: vx=%lf, vy=%lf, w0=%lf\n", vx, vy, w0);
    wheel->setMotionRect(vx, vy, w0);
  }

  return 0;
}