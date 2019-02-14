/********
*api_C.h*
********/
/**
 * \file api.h
 *
 * PROS API header provides high-level user functionality
 *
 * Contains declarations for use by typical VEX programmers using PROS.
 *
 * This file should not be modified by users, since it gets replaced whenever
 * a kernel upgrade occurs.
 *
 * Copyright (c) 2017-2018, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_API_H_
#define _PROS_API_H_

#ifdef __cplusplus
#include <cerrno>
#include <cmath>
#include <cstdbool>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#else /* (not) __cplusplus */
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#endif /* __cplusplus */

#define PROS_VERSION_MAJOR 3
#define PROS_VERSION_MINOR 1
#define PROS_VERSION_PATCH 0
#define PROS_VERSION_STRING "3.1.0"

#define PROS_ERR (INT32_MAX)
#define PROS_ERR_F (INFINITY)

#include "../include/pros/adi.h"
#include "../include/pros/colors.h"
#include "../include/pros/llemu.h"
#include "../include/pros/misc.h"
#include "../include/pros/motors.h"
#include "../include/pros/rtos.h"
#include "../include/pros/vision.h"

#ifdef __cplusplus
#include "../include/pros/adi.hpp"
#include "../include/pros/llemu.hpp"
#include "../include/pros/misc.hpp"
#include "../include/pros/motors.hpp"
#include "../include/pros/rtos.hpp"
#include "../include/pros/vision.hpp"
#endif

#endif  // _PROS_API_H_
/******************
*autonomousRed_C.h*
******************/
#ifndef _AUTONOMOUS_RED_C_H
#define _AUTONOMOUS_RED_C_H


void autonomous();

void initial_shooter_feed();
void get_pole_side_blue_cap();
void give_pole_side_blue_cap_balls_to_shooter();
void place_first_cap_on_pole();
void get_pole_side_red_cap();
void place_second_cap_on_pole();
void get_net_side_red_cap();
void place_third_cap_on_pole();
void grab_net_side_blue_cap();
void feed_net_side_balls_to_shooter();
void return_to_start();

void getCap();
void flipCap();
void putOnSmallPole();
void putOnBigPole();
void dropCap();


#endif // _AUTONOMOUS_RED_C_H
/*********
*main_C.h*
*********/
/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2018, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter,
 * more convenient naming pattern. If this isn't desired, simply comment the
 * following line out
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER
 * is pedantically correct within the PROS styleguide, but not convienent for
 * most student
 * programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use.
 *
 * For instance, you can do `4_m = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS




/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor,
 * you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently!
 *            The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace okapi;

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
/***********
*Motors_C.h*
***********/
#ifndef _MOTORS_C_C
#define _MOTORS_C_C

#define ARM_CONTROLLER      E_CONTROLLER_MASTER
#define DRIVE_CONTROLLER    E_CONTROLLER_MASTER
#define CLAW_CONTROLLER     E_CONTROLLER_MASTER

#define ARM_UP_BTN          E_CONTROLLER_DIGITAL_L1
#define ARM_DN_BTN          E_CONTROLLER_DIGITAL_L2
#define DRIVE_LEFT_STICK    E_CONTROLLER_ANALOG_LEFT_Y
#define DRIVE_RIGHT_STICK   E_CONTROLLER_ANALOG_RIGHT_Y
#define DRIVE_LEFT_STRAFE_STICK     E_CONTROLLER_ANALOG_LEFT_X
#define DRIVE_RIGHT_STRAFE_STICK    E_CONTROLLER_ANALOG_RIGHT_X
#define CLAW_OPEN_BTN       E_CONTROLLER_DIGITAL_R2
#define CLAW_CLOSE_BTN      E_CONTROLLER_DIGITAL_R1
#define CLAW_CW_BTN         E_CONTROLLER_DIGITAL_B
#define CLAW_CCW_BTN        E_CONTROLLER_DIGITAL_A

#define ARM_LEFT_PORT           10
#define ARM_RIGHT_PORT          20
#define DRIVE_LEFT_FRONT_PORT   1
#define DRIVE_LEFT_REAR_PORT    2
#define DRIVE_RIGHT_FRONT_PORT  11
#define DRIVE_RIGHT_REAR_PORT   12
#define CLAW_ROTATE_PORT        9
#define CLAW_PORT               8

#define DRIVE_FORWARD  1
#define DRIVE_BACKWARD -DRIVE_FORWARD
#define ARM_UP   1
#define ARM_DOWN -ARM_UP
#define CLAW_OPEN  1
#define CLAW_CLOSE -CLAW_OPEN
#define CLAW_CW  1
#define CLAW_CCW -CLAW_CW


void armSpeed(int speed);
void leftDrive(int speed);
void rightDrive(int speed);
void clawRotate(int direction);
void clawSpeed(int speed);

#endif // _MOTORS_C_C
/******
*PID.h*
******/
#ifndef _PID_H_
#define _PID_H_
// #warning "In PID header"

typedef struct {
	double Kp;
	double Ki;
	double Kd;
	long long error;
	long long integral;
	long long derivative;
	long long target;
	long long previousError;
	int *motorPorts;
	int numMotorPorts;
	long long startSlowingValue;
} PID_properties_t;

typedef PID_properties_t *PID_array_t;


/* Function:		generateNextPID
 * Purpose:			Updates the PID_properties_t by running through one pass of the PID algorithm
 * Argument:		prop = the property to be updated
 * Return:			the next PID_properties_t object
 */
PID_properties_t generateNextPID(PID_properties_t prop);

/* Function:		generateMovedPID
 * Purpose:			moves the target property of prop by targetDelta
 * Argument:		prop = the property to which the target will be moved
 *                  targetDelta = amount to add to prop's target
 * Return:			the updated PID_properties_t object
 */
PID_properties_t generateMovedPID(PID_properties_t prop, long long targetDelta);

/* Function:        generateRotatedDrive
 * Purpose:         rotates the robot's drive by addint the targets of right and left to target and -target respectively
 * Argument:        left = the properties of the left side of the drive train
 *                  right = the properties of the right side of the drive train
 *                  target = the amount to add to right and subtract from left
 * Return:          the array of PID_properties_t where index 0 is left and index 1 is right
 */
PID_array_t generateRotatedDrive(PID_properties_t left, PID_properties_t right, long long target);

/* Function:		atTarget
 * Purpose:			determines whether the motor has successfully moved to the target
 * Argument:		prop = the property to test
 * Return:			true if the magnitude of error is less than 5 and isStopped() is true, false therwise
 */
int atTarget(PID_properties_t prop);

/* Function:		isStopped
 * Purpose:			determines whether the motors have stopped moving based on derivative
 * Argument:		prop = the property to test
 * Return:			1 if the derivative is 0, 0 therwise
 */
int isStopped(PID_properties_t prop);

/* Function:		createPID
 * Purpose:			Generates a new PID_properties_t object using the parameters
 * Argument:		Kp = multiplier for the proportion
                    Ki = multiplier for the integral
                    Kd = multiplier for the derivative
                    motorPorts = the motor ports that are associated with the PID_properties_t
 					numMotorPorts = the length of motorPorts
					startSlowingValue = the error value where the motors will start to slow down
 * Return:			The created PID_properties_t object
 */
PID_properties_t createPID(double Kp, double Ki, double Kd, int *motorPorts, int numMotorPorts, long long startSlowingValue);

/* !EXPERIMENTAL!
 * Function:		applyRealTimeCorrection
 * Purpose:         adjusts the derivative of the PID_properties_t object so that it will be more accurate on the next move
 * Argument:        prop = the PID_properties_t object to which the algorithm will be applied
 * Return:			the next PID_properties_t object
 */
PID_properties_t applyRealTimeCorrection(PID_properties_t prop);

/* Function:        findKpid_Ziegler
 * Purpose:         find the constants for PID using Ziegler-Nichols method
 * Argument:        motorPorts = the motor ports that are associated with the PID_properties_t
                    numMotorPorts = the length of numMotorPorts
                    startSlowingValue = the error value where teh motors will start to slow down
                    target = the distance to move the motor for testing
 * Return:          the created PID_properties_t object
 */
PID_properties_t findKpid_Ziegler(int* motorPorts, int numMotorPorts, long long startSlowingValue, long long target);

/* Function:        findKpid_manual
 * Purpose:         find the constants for PID using manual method
 * Argument:        motorPorts = the motor ports that are associated with the PID_properties_t
                    numMotorPorts = the length of numMotorPorts
                    startSlowingValue = the error value where teh motors will start to slow down
                    target = the distance to move the motor for testing
 * Return:          the created PID_properties_t object
 */
PID_properties_t findKpid_manual(int* motorPorts, int numMotorPorts, long long startSlowingValue, long long target);



#endif // _PID_H_
/************
*Sensors_C.h*
************/
#ifndef _SENSORS_H_
#define _SENSORS_H_

#define PI 3.1415
#define WHEEL_DIAMETER ((4 + 4.25)/2)
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)
#define Pole_Hight_Small 23.0
#define Pole_Hight_Large 34.0
#define WALL_TO_WALL_INCHES 140.5
#define WALL_TO_WALL_MATS 6
// ~23.42 inches per mat
#define INCHES_PER_MAT (WALL_TO_WALL_INCHES / WALL_TO_WALL_MATS)
// Driving constants
#define MOTOR_COUNT_PER_REVOLUTION 4554752
#define MOTOR_COUNTS_PER_INCH  ((double) MOTOR_COUNT_PER_REVOLUTION / WHEEL_CIRCUMFERENCE)
#define MOTOR_DEGREES_PER_INCH ((double) 360 / WHEEL_CIRCUMFERENCE)
// Turning constants
#define ROBOT_ROTATION_MOTOR_COUNTS 1200
// #define ROBOT_ROTATION_MOTOR_COUNTS 988.78
#define ROBOT_ROTATION_COUNTS_PER_DEGREE (ROBOT_ROTATION_MOTOR_COUNTS / 360)
#define ROBOT_ROTATION_TURN_LEFT 1
#define ROBOT_ROTATION_TURN_RIGHT -ROBOT_ROTATION_TURN_LEFT


/* Function:		initializePID
 * Purpose:			Initializes the PID objects for the robot.
 * Argument:		N/A
 * Returns:			N/A
 */
void initializePIDs();

void initializeMotors();

/* Function:        setupMotor
 * Purpose:			Initializes the motor with raw encoder counts.
 * Argument:		port = smart port to setup
                    reversed = 1 to reverse motor
                    gearset = the gearset of the motor
 * Returns:			N/A
 */
void setupMotor(int port, int reversed, int gearset);

void moveDrivePID();

/* Function:		moveRaw
 * Purpose:			move the robot the specified number of raw counts
 * Argument:		raw = number of raw encoder counts to move
 * Returns:			N/A
 */
void moveRaw(long raw);

/* Function:		moveIn
 * Purpose:			moves the robot a specified amount of inches
 * Argument:		inches = amount of inches to move
 * Returns:			N/A
 */
void moveIn(float inches);

/* Function:		moveMats
 * Purpose:			moves the robot a specified amount of mats
 * Argument:		mats = amount of mats to move
 * Returns:			N/A
 */
void moveMats(float mats);

/* Function:		rotateTo
 * Purpose:			rotates the robot to the specified degree
 * Argument:		deg = degree to move to
 * Return:			N/A
 */
void rotateTo(float targetDeg);

long leftDrivePos();
long rightDrivePos();
long drivePos();


#endif // _SENSORS_H_
/******************
*autonomousRed_C.c*
******************/

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
/***************
*initialize_C.c*
***************/

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
**/
void initialize() {
    initializePIDs();
    initializeMotors();

 #warning "Testing for moveIn() enabled"
    // moveIn(36);
    // moveMats(1);
    rotateTo(ROBOT_ROTATION_TURN_LEFT * 360);

    printf("\n\nINIT DONE\n");

    while (1) {
    //     printf("%f | %f | %f | %f    \n",
    //             motor_get_position(1),
    //             motor_get_position(2),
    //             motor_get_position(11),
    //             motor_get_position(12));
    //
    //     delay(1000);
    }
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
/***********
*Motors_C.c*
***********/

void armSpeed(int speed) {
    motor_move(ARM_LEFT_PORT, speed);
    motor_move(ARM_RIGHT_PORT, speed);
}

void leftDrive(int speed) {
    motor_move(DRIVE_LEFT_FRONT_PORT, speed);
    motor_move(DRIVE_LEFT_REAR_PORT, speed);
}

void rightDrive(int speed) {
    motor_move(DRIVE_RIGHT_FRONT_PORT, speed);
    motor_move(DRIVE_RIGHT_REAR_PORT, speed);
}

void clawRotate(int direction) {
    motor_move(CLAW_ROTATE_PORT, direction);
}

void clawSpeed(int speed) {
    motor_move(CLAW_PORT, speed);
}

void t() {
    armSpeed(10);
}
/**************
*opcontrol_C.c*
**************/
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void armControl() {
    int _armSpeed = 127 * ARM_UP * controller_get_digital(ARM_CONTROLLER, ARM_UP_BTN);
    _armSpeed += 127 * ARM_DOWN * controller_get_digital(ARM_CONTROLLER, ARM_DN_BTN);

    printf("%d\n", _armSpeed);
    armSpeed(_armSpeed);
}

void driveControl() {
    int leftY  = DRIVE_FORWARD * controller_get_analog(DRIVE_CONTROLLER, DRIVE_LEFT_STICK);
    int rightY = DRIVE_FORWARD * controller_get_analog(DRIVE_CONTROLLER, DRIVE_RIGHT_STICK);
    int leftX  = DRIVE_FORWARD * controller_get_analog(DRIVE_CONTROLLER, DRIVE_LEFT_STRAFE_STICK);
    int rightX = DRIVE_FORWARD * controller_get_analog(DRIVE_CONTROLLER, DRIVE_RIGHT_STRAFE_STICK);

//---- TANK DRIVE
    // int driveScaledL = (leftY * leftY * leftY) / (127 * 127);
    // int driveScaledR = (rightY * rightY * rightY) / (127 * 127);
    // leftDrive(driveScaledL);
    // rightDrive(driveScaledR);

//---- ARCADE DRIVE [R] (RIGHT DRIVE, LEFT TURN)
    // int turningScaled = (leftX * leftX * leftX) / (127 * 127);
    // int driveScaled = (rightY * rightY * rightY) / (127 * 127);
    // leftDrive(driveScaled + turningScaled);
    // rightDrive(driveScaled - turningScaled);

//---- ARCADE DRIVE [L] (LEFT DRIVE, RIGHT TURN)
    int turningScaled = (rightX * rightX * rightX) / (127 * 127);
    int driveScaled = (leftY * leftY * leftY) / (127 * 127);
    leftDrive(driveScaled - turningScaled);
    rightDrive(driveScaled + turningScaled);
}

void clawControl() {
    int _clawSpeed = 127 * CLAW_OPEN * controller_get_digital(CLAW_CONTROLLER, CLAW_OPEN_BTN);
    _clawSpeed += 127 * CLAW_CLOSE * controller_get_digital(CLAW_CONTROLLER, CLAW_CLOSE_BTN);

    int clawRotateSpeed = 127 * CLAW_CW * controller_get_digital(CLAW_CONTROLLER, CLAW_CW_BTN);
    clawRotateSpeed += 127 * CLAW_CCW * controller_get_digital(CLAW_CONTROLLER, CLAW_CCW_BTN);

    clawSpeed(_clawSpeed);
    clawRotate(clawRotateSpeed);
}

void opcontrol() {
    while (1) {
        armControl();
        driveControl();
        clawControl();
    }
}
/******
*PID.c*
******/
// #warning "In PID source"

void p(int n, PID_properties_t prop) {
    printf("@id %d: %d, %d; %d | ", n, prop.motorPorts[0], prop.motorPorts[1], prop.motorPorts);
}

PID_properties_t generateNextPID(PID_properties_t prop) {
	int speed, i;
 // p(0, prop);
    int avgPosition = 0;
    for (int i = 0; i < prop.numMotorPorts; i++)
        avgPosition += motor_get_position(prop.motorPorts[i]);
    avgPosition /= prop.numMotorPorts;

	prop.error = prop.target - avgPosition;
	prop.integral += prop.error;

	if (prop.error == 0)
		prop.integral = 0;
	if (abs(prop.error) < prop.startSlowingValue)
		prop.integral = 0;

	prop.derivative = prop.error - prop.previousError;
	prop.previousError = prop.error;

	speed = prop.Kp * prop.error + prop.Ki * prop.integral + prop.Kd * prop.derivative;

    if (speed > 127)
        speed = 127;
    else if (speed < -127)
        speed = -127;

	for (i = 0; i < prop.numMotorPorts; ++i)
		motor_move(prop.motorPorts[i], speed);

    return prop;
}

PID_properties_t generateMovedPID(PID_properties_t prop, long long targetDelta) {
    prop.target += targetDelta;
    prop.error += targetDelta;
    return prop;
}

PID_array_t generateRotatedDrive(PID_properties_t left, PID_properties_t right, long long target) {
    left = generateMovedPID(left, -target);
    right = generateMovedPID(right, target);

    do {
        left = generateNextPID(left);
        right = generateNextPID(right);
    } while (!atTarget(left) || !atTarget(right));

    PID_array_t drive = malloc(sizeof(PID_properties_t) * 2);
    drive[0] = left;
    drive[1] = right;
    return drive;
}

int atTarget(PID_properties_t prop) {
    return isStopped(prop) && abs(prop.error) < 5;
}

int isStopped(PID_properties_t prop) {
    return prop.derivative == 0;
}

PID_properties_t createPID(double Kp, double Ki, double Kd, int *motorPorts, int numMotorPorts, long long startSlowingValue) {
	PID_properties_t prop;

	prop.Kp = Kp;
	prop.Ki = Ki;
	prop.Kd = Kd;
	prop.numMotorPorts = numMotorPorts;
	prop.motorPorts = motorPorts;
	prop.startSlowingValue = startSlowingValue;

    prop.target = 0;
    prop.error = 0;
    prop.previousError = 0;
    prop.derivative = 0;
    prop.integral = 0;

	return prop;
}

PID_properties_t applyRealTimeCorrection(PID_properties_t prop) {
    if (isStopped(prop)) {
        if (prop.error < 0) { // robot went too far; derivative is too high
            prop.Ki -= .0000000001;
        }
        else if (prop.error > 0) { // robot did not go far enough; derivative is too low
            prop.Ki += .0000000001;
        }
    }

    return prop;
}

PID_properties_t findKpid_Ziegler(int* motorPorts, int numMotorPorts, long long startSlowingValue, long long target) {
    PID_properties_t prop = createPID(.05, 0, 0, motorPorts, numMotorPorts, startSlowingValue);

    int dir = 1;

    while (1) {
        prop.target = target * dir;
        prop.error = target; // error can only be at max to begin with

        int lastDir = 2 * dir - 1;
        long lastPeriodMeasured; // first measurement will be bad
        for (int i = 0; i < 1000 && !atTarget(prop);) {
            prop = generateNextPID(prop);

            // is this pass the start of a new period measurement?
            int currentDir = (prop.derivative > 0)? 1: -1;

            // if (changed direction of motion && direction starts new period)
            if (currentDir == -lastDir && currentDir == 2 * dir - 1) {
                // set priod to be new measured value
                long now = millis();
                long period = now - lastPeriodMeasured;
                lastPeriodMeasured = now;

                // print new period to console
                printf("Pu=%6d | Ku=%5.2f\n", period, prop.Kp);
            }

            lastDir = currentDir;

            // determine if the robot is no long longer going to move
            if (isStopped(prop))
                i++;
            else
                i = 0;
        }

        prop.Kp += .05;

        dir = 1 - dir; // switch directions
        delay(1500); // wait for robot to settle
    }
}

PID_properties_t findKpid_manual(int* motorPorts, int numMotorPorts, long long startSlowingValue, long long target) {
    PID_properties_t prop = createPID(0, 0, 0, motorPorts, numMotorPorts, startSlowingValue);

    int dir = 1; // 1 for forward, 0 for return to starts. switch w/ dir = 1-dir;

    // the Kp, Ki, Kd tuning constants, acceptable errors
    double tuningVars[3][2] = { {.03,            20}, // for Kp
                                {.002,          10}, // for Kd
                                {.0000000001,   5}}; // for Ki

    for (int i = 0; i < 3; i++) {
        do {
            prop.target = target * dir;
            prop.error = target; // error can only be at max to begin with

printf("> ");
            switch (i) {
                case 0:
                    prop.Kp += tuningVars[0][0];
printf("p%f | ", prop.Kp);
                    break;
                case 1:
                    prop.Kd += tuningVars[1][0];
printf("d%f | ", prop.Kd);
                    break;
                case 2:
                    prop.Ki += tuningVars[2][0];
printf("i%f | ", prop.Ki);
                    break;
            }

printf("%d | %f | %f -> ", i, prop.error, prop.Kp);
            for (int j = 0; j < 1000 && abs(prop.error) > tuningVars[i][1];) {
                prop = generateNextPID(prop);

                // determine if the robot is no long longer going to move
                if (isStopped(prop))
                    j++;
                else
                    j = 0;
printf(" j%d|d%f", j, prop.derivative);
            }
printf("%f | %f\n", prop.error, prop.Kp);

            dir = 1 - dir; // switch directions
            delay(1500); // wait for robot to settle
        } while (abs(prop.error) > tuningVars[i][1]);
    }


    // print out values
    while (1)
        printf("Kpid=%5.2f | %5.2f | %5.2f\n", prop.Kp, prop.Ki, prop.Kd);
}
/************
*Sensors_C.c*
************/

PID_properties_t wheelsLeft, wheelsRight;
int leftWheelPorts[] = {11, 12};
int rightWheelPorts[] = {1, 2};

void initializePIDs() {
    setupMotor(DRIVE_RIGHT_FRONT_PORT, 1, E_MOTOR_GEARSET_18);
    setupMotor(DRIVE_RIGHT_REAR_PORT, 1, E_MOTOR_GEARSET_18);
    setupMotor(DRIVE_LEFT_FRONT_PORT, 0, E_MOTOR_GEARSET_18);
    setupMotor(DRIVE_LEFT_REAR_PORT, 0, E_MOTOR_GEARSET_18);


    float driveKp = 0.2;
    float driveKi = 0.0000018; // orig = 0.00000035
    float driveKd = 0.0001;

    wheelsLeft  = createPID(driveKp, driveKi, driveKd, leftWheelPorts,  2, 20);
    wheelsRight = createPID(driveKp, driveKi, driveKd, rightWheelPorts, 2, 20);
 printf("PID init ports: L = %d,%d | R = %d, %d\n", leftWheelPorts[0], leftWheelPorts[1], rightWheelPorts[0], rightWheelPorts[1]);
 printf("PID init: L = %d,%d | R = %d, %d\n", wheelsLeft.motorPorts[0], wheelsLeft.motorPorts[1], wheelsRight.motorPorts[0], wheelsRight.motorPorts[1]);
}

void initializeMotors() {
    setupMotor(ARM_LEFT_PORT, 1, E_MOTOR_GEARSET_18);
    setupMotor(ARM_RIGHT_PORT, 0, E_MOTOR_GEARSET_18);

    setupMotor(CLAW_ROTATE_PORT, 0, E_MOTOR_GEARSET_18);
    setupMotor(CLAW_PORT, 0, E_MOTOR_GEARSET_18);
}

void setupMotor(int port, int reversed, int gearset) {
	motor_set_gearing(port, gearset);
	motor_set_reversed(port, reversed);
	motor_set_encoder_units(port, E_MOTOR_ENCODER_DEGREES);
    motor_set_brake_mode(port, E_MOTOR_BRAKE_COAST);
}

void moveDrivePID() {
    while (!atTarget(wheelsLeft) && !atTarget(wheelsRight)) {
    // for (int i = 0; i < 100; i++) {
 // printf("<LEFT> ");
        wheelsLeft = generateNextPID(wheelsLeft);
 // printf("<RIGHT> ");
        wheelsRight = generateNextPID(wheelsRight);
 // printf("error: %lld, %lld\n", wheelsLeft.error, wheelsRight.error);
 // printf("speed: %lld, %lld\n", wheelsLeft.previousError, wheelsRight.previousError);
 printf("encoder: %.2f, %.2f\n", motor_get_position(1), motor_get_position(11));
 printf("\n");
    }
}

void moveRaw(long raw) {
    wheelsLeft = generateMovedPID(wheelsLeft, raw);
    wheelsRight = generateMovedPID(wheelsRight, raw);

    moveDrivePID();
}
void moveIn(float inches) {
    moveRaw(inches * MOTOR_DEGREES_PER_INCH);
}
void moveMats(float mats) {
    moveIn(mats * INCHES_PER_MAT);
}

void rotateTo(float targetDeg) {
    long targetRaw = targetDeg * ROBOT_ROTATION_COUNTS_PER_DEGREE;

    wheelsLeft = generateMovedPID(wheelsLeft, targetRaw);
    wheelsRight = generateMovedPID(wheelsRight, -targetRaw);

    moveDrivePID();
}

long leftDrivePos() {
	return (motor_get_position(wheelsLeft.motorPorts[0]) + motor_get_position(wheelsLeft.motorPorts[1])) / 2;
}
long rightDrivePos() {
	return (motor_get_position(wheelsRight.motorPorts[0]) + motor_get_position(wheelsRight.motorPorts[1])) / 2;
}
long drivePos() {
	return (leftDrivePos() + rightDrivePos()) / 2;
}
