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
	// Feed shooter
	moveMats(.75);
	moveArmTo(CAP_FLIP_HEIGHT); // help the claw disengage
	moveMats(-1.7);
	moveArmTo(CAP_HEIGHT);
	rotateTo(ROBOT_ROTATION_TURN_RIGHT * 90);
	// At cap
	moveMats(2);
	moveMats(-1);
	moveArmTo(CAP_FLIP_HEIGHT);
	
	// Rotate claw
	clawRotate(127);
	delay(.1);
	clawRotate(0);
	
	moveArmTo(POLE_PREPARE_HEIGHT);
	rotateTo(ROBOT_ROTATION_TURN_RIGHT * 90);
	// Facing pole
	moveMats(.1); // Line up with the pole
	moveArmTo(POLE_BIG_HEIGHT);
	moveMats(-.5); // Disengage the cap
}


void preload_shooter() {
    moveMats(.75);
	moveArmTo(CAP_FLIP_HEIGHT);
    moveMats(-1.7);
    rotateTo(ROBOT_ROTATION_TURN_RIGHT * 65);
}

void get_pole_side_blue_cap() {
    moveMats(2.5);
    getCap();
}

void give_pole_side_blue_cap_balls_to_shooter() {
    rotateTo(ROBOT_ROTATION_TURN_LEFT * 140);
    moveMats(2.5);
    flipCap();
}

void place_first_cap_on_pole() {
    // moveMats(-1.5); // moveMats(-2.5); (OLD)
    // rotateTo(ROBOT_ROTATION_TURN_LEFT * 90);
    // moveMats(.5);
    putOnSmallPole();
}

void get_pole_side_red_cap() {
    // moveMats(-.5);
    //back in same starting pos
    rotateTo(ROBOT_ROTATION_TURN_RIGHT * 90);
    moveMats(0.5);
    rotateTo(ROBOT_ROTATION_TURN_RIGHT * 90);
    moveMats(1.5);// moveMats(2);
    getCap();
}

void place_second_cap_on_pole() {
    rotateTo(ROBOT_ROTATION_TURN_RIGHT * 180);
    moveMats(1);
    rotateTo(ROBOT_ROTATION_TURN_LEFT * 90);
    moveMats(1); // moveMats(1.5);
    putOnSmallPole();
}

void get_net_side_red_cap() {
    // moveMats(-0.5);
    rotateTo(ROBOT_ROTATION_TURN_RIGHT * 90);
    moveMats(1);
    rotateTo(ROBOT_ROTATION_TURN_RIGHT * 90);
    moveMats(3);
    rotateTo(ROBOT_ROTATION_TURN_RIGHT * 90);
    moveMats(1.5);// moveMats(2);
    getCap();
}

void place_third_cap_on_pole() {
    rotateTo(180);
    moveMats(2);
    rotateTo(ROBOT_ROTATION_TURN_RIGHT * 90);
    moveMats(0.5);
    rotateTo(ROBOT_ROTATION_TURN_LEFT  * 90);
    // moveMats(0.5);
    putOnBigPole();
}

void grab_net_side_blue_cap() {
    rotateTo(ROBOT_ROTATION_TURN_LEFT * 90);
    moveMats(.5);
    rotateTo(ROBOT_ROTATION_TURN_LEFT * 90);
    moveMats(1);
    rotateTo(ROBOT_ROTATION_TURN_LEFT * 90);
    moveMats(.5);// moveMats(1);
    getCap();
}

void return_to_start() {
    // feed net-side balls to shooter
    moveMats(-1);
    rotateTo(ROBOT_ROTATION_TURN_LEFT * 90);
    moveMats(1);
    flipCap();
    // return to start
    rotateTo(ROBOT_ROTATION_TURN_LEFT * 90);
    moveMats(2);
}

void getCap(){
    // openClaw();
    moveArmTo(CAP_HEIGHT);
    moveMats(CAP_LINE_UP);
	moveArmTo(CAP_FLIP_HEIGHT);
    // closeClaw();
}

void putOnSmallPole() {
    // move cap above pole to prepare capping
    moveArmTo(POLE_PREPARE_HEIGHT);
    // line up cap
    moveMats(POLE_SMALL_LINE_UP);
    moveArmTo(POLE_SMALL_HEIGHT);
    // openClaw();
    moveMats(-POLE_SMALL_LINE_UP);
}

void putOnBigPole() {
    // move cap above pole to prepare capping
    moveArmTo(POLE_PREPARE_HEIGHT);
    // line up cap
    moveMats(POLE_BIG_LINE_UP);
    moveArmTo(POLE_BIG_HEIGHT);
    // openClaw();
    moveMats(-POLE_BIG_LINE_UP);
}

void flipCap(){
    // arm should be more than 1 cap radius above ground; maybe add check for arm > CAP_FLIP_HEIGHT
    moveArmTo(CAP_FLIP_HEIGHT);
    claw180Rotate();
}

void dropCap(){
    // make slightly above ground so cap falls out of claw
    moveArmTo(CAP_HEIGHT);
    // openClaw();
    moveMats(-CAP_LINE_UP);
}
