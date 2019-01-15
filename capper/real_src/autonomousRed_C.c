#include "../include/main_C.h"

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

void autonomous() {
    // start facing other robot, twords top
    initial_shooter_feed();

    get_pole_side_blue_cap();
    give_pole_side_blue_cap_balls_to_shooter();
    place_first_cap_on_pole();

    get_pole_side_red_cap();
    place_second_cap_on_pole();
    get_net_side_red_cap();

    place_third_cap_on_pole();
    grab_net_side_blue_cap();
    feed_net_side_balls_to_shooter();

    return_to_start();
}


void initial_shooter_feed() {
    moveMats(1.5);
    moveMats(-2.5);
    rotateTo(-90);
}

void get_pole_side_blue_cap() {
    moveMats(2);
    getCap();
}

void give_pole_side_blue_cap_balls_to_shooter() {
    moveMats(-2);
    rotateTo(90);
    moveMats(3);
    flipCap();
}

void place_first_cap_on_pole() {
    moveMats(-1.5); // moveMats(-2.5); (OLD)
    rotateTo(90);
    moveMats(.5);
    putOnSmallPole();
}

void get_pole_side_red_cap() {
    moveMats(-.5);
    //back in same starting pos
    rotateTo(-90);
    moveMats(0.5);
    rotateTo(-90);
    moveMats(2);
    getCap();
}

void place_second_cap_on_pole() {
    rotateTo(180);
    moveMats(1);
    rotateTo(90);
    moveMats(1.5);
    putOnSmallPole();
}

void get_net_side_red_cap() {
    moveMats(-0.5);
    rotateTo(-90);
    moveMats(1);
    rotateTo(-90);
    moveMats(3);
    rotateTo(-90);
    moveMats(2);
    getCap();
}

void place_third_cap_on_pole() {
    rotateTo(180);
    moveMats(2);
    rotateTo(90);
    moveMats(0.5);
    rotateTo(-90);
    moveMats(0.5);
    putOnBigPole();
}

void grab_net_side_blue_cap() {
    rotateTo(-90);
    moveMats(1.5);
    rotateTo(-90);
    moveMats(1.5);
    getCap();
}

void feed_net_side_balls_to_shooter() {
    moveMats(-1);
    rotateTo(-90);
    moveMats(3);
    rotateTo(180);
    moveMats(1);
    flipCap();
}

void return_to_start() {
    dropCap();
    moveMats(-1);
    rotateTo(90);
}

#warning "Auton getCap() not complete"
void getCap(){
    // openClaw();
    // moveArmTo(CAP_HEIGHT);
    // closeClaw();
}

#warning "Auton putOnSmallPole() not complete"
void putOnSmallPole() {
    // // move cap above pole to prepare capping
    // moveArmTo(POLE_CAP_PREPARE);
    // // line up cap
    // moveIn(POLE_SMALL_LINE_UP);
    // moveArmTo(POLE_SMALL_HEIGHT);
    // openClaw();
    // moveIn(-POLE_SMALL_LINE_UP);
}

#warning "Auton putOnBigPole() not complete"
void putOnBigPole() {
    // // move cap above pole to prepare capping
    // moveArmTo(POLE_PREPARE_HEIGHT);
    // // line up cap
    // moveIn(POLE_BIG_LINE_UP);
    // moveArmTo(POLE_BIG_HEIGHT);
    // openClaw();
    // moveIn(-POLE_BIG_LINE_UP);
}

#warning "Auton flipCap() not complete"
void flipCap(){
    // // arm should be more than 1 cap radius above ground; maybe add check for arm > CAP_FLIP_HEIGHT
    // moveArmTo(CAP_FLIP_HEIGHT);
    // flipCap();
}

#warning "Auton dropCap() not complete"
void dropCap(){
    // // make slightly above ground so cap falls out of claw
    // moveArmTo(CAP_FLIP_HEIGHT);
    // openClaw();
}
