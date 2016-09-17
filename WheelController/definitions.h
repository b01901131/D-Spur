#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define PI 3.1415926

#define FULL 0x0
#define HALF 0x4
#define QUARTER 0x2
#define EIGHTH 0x6
#define SIXTEENTH 0x7

#define ANGULAR_ACCELERATION 2
//#define CMD_LEN 18
#define CMD_LEN 10

#define DEBUG true

const double MAX_ANG_VELOCITY[5] = 
    {PI/180*3600, PI/180*1800, PI/180*900, PI/180*450, PI/180*225};

const double MIN_ANG_VELOCITY[5] = 
    {PI/180*1800, PI/180*900,  PI/180*450, PI/180*225, PI/180*112.5*0};

#endif
