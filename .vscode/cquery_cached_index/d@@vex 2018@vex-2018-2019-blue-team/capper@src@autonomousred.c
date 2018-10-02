#include "main.h"

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
 // start facing other robot, twords top
 #define MAT 22.1
 moveI(MAT);
 moveI(-MAT*2);
 turn 90 degrees right
 moveI(MAT*2);
 getCap();
 moveI(-MAT*2)
 turn 90 degrees left
 moveI(MAT*2);
 flipCap();
 moveI(-MAT*1.5);
 turn 90 degrees left
 moveI(MAT*.5);
 putOnPole();
 moveI()-MAT*.5);
 turn right 90 degrees
 moveI(MAT*.5);
 turn right 90 degrees
 moveI(MAT*2);
 getCap();
 turn 180 degrees
 moveI(MAT);
 turn left 90
 moveI(MAT*1.5);
 putOnPole();
 moveI(-MAT*.5);
 turn right 90
 moveI(MAT);
 turn right 90
 moveI(MAT*3);
 turn right 90
 moveI(MAT*2);
 grabCap();
 turn 180
 moveI(MAT*2.5);
 turn left 90
 moveI(MAT*.5);
 turn right 90
 placeCap();
 turn right 90
 moveI(MAT*1.5);
 turn right 90
 moveI(MAT*1.5);
 grabCap();
 moveI(-MAT);
 turn right 90
 moveI(MAT*2);
 flipCap();
 moveI(MAT*2);
 dropCap();
 moveI(-MAT);
 turn left 90


void autonomous() {}
