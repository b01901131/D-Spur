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
  SerialPortController* serial_port_controller = new SerialPortController("/dev/cu.wchusbserial1410");
  //SerialPortController* serial_port_controller = NULL;

  WheelController* wheel = new WheelController(serial_port_controller);

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
      continue;
    }
    printf("main: vx=%lf, vy=%lf, w0=%lf\n", vx, vy, w0);
    wheel->setMotionRect(vx, vy, w0);
  }

  return 0;
}
