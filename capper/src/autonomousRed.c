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
 define MAT = 22.1
 drive foward MAT inches
 drive backwords MAT*2 inches
 turn 90 degrees right
 drive foward MAT*2
 grab cap
 drive backwords MAT*2 inches
 turn 90 degrees left
 drive foward MAT*2 inches
 flip cap
 drive backwords MAT*1.5 inches
 turn 90 degrees left
 drive fowards MAT*.5
 put on pole
 drive backwords MAT*.5
 turn right 90 degrees
 drive foward MAT*.5 inches
 turn right 90 degrees
 drive foward MAT*2 inches
 grab cap
 turn 180 degrees
 drive foward MAT
 turn left 90
 drive foward MAT*1.5
 put cap
 drive backwords MAT*.5
 turn right 90
 drive foward MAT
 turn right 90
 drive foward MAT*3
 turn right 90
 drive foward MAT*2
 grab cap
 turn 180
 drive foward MAT*2.5
 turn left 90
 drive foward MAT*.5
 turn right 90
 place cap
 turn right 90
 drive foward MAT*1.5
 turn right 90
 drive foward MAT*1.5
 grab cap
 drive backwords MAT
 turn right 90
 drive foward MAT*2
 flip cap
 drive foward MAT*2
 drop cap
 drivebackwords MAT
 turn left 90


void autonomous() {}
