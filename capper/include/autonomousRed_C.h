#ifndef _AUTONOMOUS_RED_C_H
#define _AUTONOMOUS_RED_C_H

#include "PID.h"

// NOTE: line up vars measured in mats
#define CAP_LINE_UP     0
#define CAP_HEIGHT      10
#define CAP_FLIP_HEIGHT (CAP_HEIGHT + 100)

#define POLE_PREPARE_HEIGHT (CAP_HEIGHT + 1200)

#define POLE_SMALL_LINE_UP 0
#define POLE_SMALL_HEIGHT  (CAP_HEIGHT + 800)

#define POLE_BIG_LINE_UP 0
#define POLE_BIG_HEIGHT  (CAP_HEIGHT + 1100)

int capFlipState = -1;

void autonomous();

void preload_shooter();
void get_pole_side_blue_cap();
void give_pole_side_blue_cap_balls_to_shooter();
void place_first_cap_on_pole();
void get_pole_side_red_cap();
void place_second_cap_on_pole();
void get_net_side_red_cap();
void place_third_cap_on_pole();
void grab_net_side_blue_cap();
void return_to_start();

void getCap();
void flipCap();
void putOnSmallPole();
void putOnBigPole();
void dropCap();


#endif // _AUTONOMOUS_RED_C_H
