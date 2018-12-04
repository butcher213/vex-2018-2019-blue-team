// #include "../include/autonomousRed_C.h"
// #include "../include/Sensors_C.h"
#include "../include/main_C.h"
// #ifdef _PID_H_
// #warning "AUTON R: PID DEFINED"
// #endif

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
**/

 /*
 moveI(MAT_Size);
 moveI(-MAT_Size*2);
 turn 90 degrees right
 moveI(MAT_Size*2);
 getCap();
 moveI(-MAT_Size*2)
 turn 90 degrees left
 moveI(MAT_Size*2);
 flipCap();
 moveI(-MAT_Size*1.5);
 turn 90 degrees left
 moveI(MAT_Size*.5);
 putOnPole();
 moveI()-MAT_Size*.5);
 turn right 90 degrees
 moveI(MAT_Size*.5);
 turn right 90 degrees
 moveI(MAT_Size*2);
 getCap();
 turn 180 degrees
 moveI(MAT_Size);
 turn left 90
 moveI(MAT_Size*1.5);
 putOnPole();
 moveI(-MAT_Size*.5);
 turn right 90
 moveI(MAT_Size);
 turn right 90
 moveI(MAT_Size*3);
 turn right 90
 moveI(MAT_Size*2);
 grabCap();
 turn 180
 moveI(MAT_Size*2.5);
 turn left 90
 moveI(MAT_Size*.5);
 turn right 90
 placeCap();
 turn right 90
 moveI(MAT_Size*1.5);
 turn right 90
 moveI(MAT_Size*1.5);
 grabCap();
 moveI(-MAT_Size);
 turn right 90
 moveI(MAT_Size*2);
 flipCap();
 moveI(MAT_Size*2);
 dropCap();
 moveI(-MAT_Size);
 turn left 90
*/


void autonomous() {

    /*
    // start facing other robot, twords top
// give preload to shooter
    moveIn(MAT_Size*1.5);
    moveIn((-MAT_Size)*2.5);
    rotateTo(-90);

// get pole-side-blue cap
    moveIn(MAT_Size*2);
    getCap();

// give pole-side-blue cap balls to shooter
    moveIn(-MAT_Size*2);
    rotateTo(90);
    moveIn(MAT_Size*3);
    flipCap();

// place first cap on pole
    moveIn(-MAT_Size*1.5); // moveIn(-MAT_Size*2.5); (OLD)
    rotateTo(90);
    moveIn(MAT_Size*.5);
    putOnPole();

// get pole-side-red cap
    moveIn(-MAT_Size*.5);
    //back in same startinbg pos
    rotateTo(-90);
    moveIn(MAT_Size*.5);
    rotateTo(-90);
    moveIn(MAT_Size*2);
    getCap();

// place second cap on pole
    rotateTo(180);
    moveIn(MAT_Size);
    rotateTo(90);
    moveIn(MAT_Size*1.5);
    putOnPole();

// get net-side-red cap
    moveIn(-MAT_Size*.5);
    rotateTo(-90);
    moveIn(MAT_Size);
    rotateTo(-90);
    moveIn(MAT_Size*3);
    rotateTo(-90);
    moveIn(MAT_Size*2);
    getCap();

// place third cap on pole
    rotateTo(180);
    moveIn(MAT_Size*2);
    rotateTo(90);
    moveIn(MAT_Size*.5);
    rotateTo(-90);
    moveIn(MAT_Size *.5);
    putOnBigPole();

// grab net-side-blue cap
    rotateTo(-90);
    moveIn(MAT_Size*1.5);
    rotateTo(-90);
    moveIn(MAT_Size*1.5);
    grabCap();

// give net-side-blue balls to shooter
    moveIn(-MAT_Size);
    rotateTo(-90);
    moveIn(MAT_Size*3);
    rotateTo(180);
    moveIn(MAT_Size*1);
    flipCap();

//
    dropCap();
    moveIn(-MAT_Size);
    rotateTo(90);
    */
}
