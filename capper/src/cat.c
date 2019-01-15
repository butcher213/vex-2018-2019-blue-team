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
/******
*PID.h*
******/
#ifndef _PID_H_
#define _PID_H_

typedef struct {
	double Kp;
	double Ki;
	double Kd;
    int speed;
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
 * Purpose:			updates the PID_properties_t by running through one pass of the PID algorithm
 * Argument:		prop = the property to be updated
 * Return:			the next PID_properties_t object
 */
PID_properties_t generateNextPID(PID_properties_t prop);

/* Function:        calculateError
 * Purpose:         calculates error for prop (distance from target)
 * Argument:        prop = the property which error will be calculated for
 * Return:          the calculated error of prop
 */
int calculateError(PID_properties_t prop);

/* Function:        generateNextSSPID
 * Purpose:         updates the system of synchronized PIDs by running throught one pass of the PID algorithm
 * Argument:        pids = the list of PIDs in the SSPIDs
 *                  length = the number of PIDs in the system
 * Return:          the updated SSPIDs
 *
 * Note:            * The SSPIDs should be initiated as a PID_array_t, then passed into this function to update them.
 *                      The code should loop through the array to initilize and set targets.
 */
PID_array_t generateNextSSPID(PID_array_t pids, int length);

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



#endif // _PID_H_
/************
*Sensors_C.h*
************/
#ifndef _SENSORS_H_
#define _SENSORS_H_

#define PI 3.1415
#define WHEEL_DIAMETER ((4 + 4.25)/2)
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)
#define MOTOR_COUNT_PER_REVOLUTION 4554752
#define MOTOR_COUNTS_PER_INCH ((double) MOTOR_COUNT_PER_REVOLUTION / WHEEL_CIRCUMFERENCE)
#define Pole_Hight_Small 23.0
#define Pole_Hight_Large 34.0
#define WALL_TO_WALL_INCHES 140.5
#define WALL_TO_WALL_MATS 6
// ~23.42 inches per mat
#define INCHES_PER_MAT (WALL_TO_WALL_INCHES / WALL_TO_WALL_MATS)


/* Function:		initializePID
 * Purpose:			Initializes the PID objects for the robot.
 * Argument:		N/A
 * Returns:			N/A
 */
void initializePIDs();

/* Function:        setupMotor
 * Purpose:			Initializes the motor with raw encoder counts.
 * Argument:		port = smart port to setup
                    reversed = 1 to reverse motor
                    gearset = the gearset of the motor
 * Returns:			N/A
 */
void setupMotor(int port, int reversed, int gearset);

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

#warning "Testing for moveIn() enabled"
    moveIn(24);

    printf("\n\nDONE\n");

    while (1);
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
void opcontrol() {

}
/******
*PID.c*
******/

PID_properties_t generateNextPID(PID_properties_t prop) {
    prop.error = calculateError(prop);
 /* =================================================
  * = Replaced with calculateError(), needs testing =
  * =================================================

 printf("1: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
    int avgPosition = 0;
    for (int i = 0; i < prop.numMotorPorts; i++)
        avgPosition += motor_get_position(prop.motorPorts[i]);
    avgPosition /= prop.numMotorPorts;

 printf("2: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
 	prop.error = prop.target - avgPosition;
 */

 printf("3: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
	if (abs(prop.error) <= prop.startSlowingValue)
		prop.integral = 0;
    else
    	prop.integral += prop.error;

 printf("4: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
	prop.derivative = prop.error - prop.previousError;
	prop.previousError = prop.error;

 printf("5: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
	prop.speed = prop.Kp * prop.error + prop.Ki * prop.integral + prop.Kd * prop.derivative;

 printf("6: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
    if (prop.speed > 127)
        prop.speed = 127;
    else if (prop.speed < -127)
        prop.speed = -127;

 printf("7: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
    // printf("motor ports: ");
	for (int i = 0; i < prop.numMotorPorts; i++) {
		motor_move(prop.motorPorts[i], prop.speed);
        // printf("%d(%d), ", prop.motorPorts[i], i);
    }
    // printf("\n");

 printf("8: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
    return prop;
}

int calculateError(PID_properties_t prop) {
 printf("1: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);

    int avgPosition = 0;
    for (int i = 0; i < prop.numMotorPorts; i++)
        avgPosition += motor_get_position(prop.motorPorts[i]);
    avgPosition /= prop.numMotorPorts;

 printf("2: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
	return prop.target - avgPosition;
}

#warning "Untested function: generateNextPID()"
PID_array_t generateNextSSPID(PID_array_t pids, int length) {
    // calculate errors of each PID
    int totalErrors[length];
    for (int i = 0; i < length; i++)
        totalErrors[i] = calculateError(pids[i]);

    // calulate each PID's distance sum from other PIDs
    for (int i = 0; i < length; i++) {
        // add on errors from other PIDs in system
        for (int j = 0; j < length; j++) {
            if (i == j) // no need to calculate for current PID
                continue;

            int dist = pids[i].error - pids[j].error;
            // add dist to each other PID in error
            totalErrors[i] += dist;
        }
    }

    // store total errors back into their respective PID
    for (int i = 0; i < length; i++)
        pids[i].error = totalErrors[i];

    // calculate PID speed normally from here
 #warning "[PID.c ~92] SSPID not implemented"
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

    prop.speed = 0;
    prop.error = 0;
    prop.integral = 0;
    prop.derivative = 0;
    prop.target = 0;
    prop.previousError = 0;

	prop.motorPorts = motorPorts;
	prop.numMotorPorts = numMotorPorts;
	prop.startSlowingValue = startSlowingValue;

    // printf("ports: %d, %d\n", prop.motorPorts[0], prop.motorPorts[1]);
	return prop;
}
/************
*Sensors_C.c*
************/

PID_properties_t leftWheels, rightWheels;

void initializePIDs() {
    setupMotor(1, 1, E_MOTOR_GEARSET_18);
    setupMotor(2, 1, E_MOTOR_GEARSET_18);
    setupMotor(11, 0, E_MOTOR_GEARSET_18);
    setupMotor(12, 0, E_MOTOR_GEARSET_18);

    int leftWheelPorts[] = {11, 12};
    int rightWheelPorts[] = {1, 2};
    
    float driveKp = 0.2;
    float driveKi = 0.00000035;
    float driveKd = 0.0001;

    leftWheels  = createPID(driveKp, driveKi, driveKd, leftWheelPorts,  2, 20);
    rightWheels = createPID(driveKp, driveKi, driveKd, rightWheelPorts, 2, 20);
}

void setupMotor(int port, int reversed, int gearset) {
	motor_set_gearing(port, gearset);
	motor_set_reversed(port, reversed);
	motor_set_encoder_units(port, E_MOTOR_ENCODER_COUNTS);
    motor_set_brake_mode(port, E_MOTOR_BRAKE_COAST);
}

void moveRaw(long raw) {
    leftWheels = generateMovedPID(leftWheels, raw);
    rightWheels = generateMovedPID(rightWheels, raw);

    while (!atTarget(leftWheels) && !atTarget(rightWheels)) {
 printf("<LEFT> ");
        leftWheels = generateNextPID(leftWheels);
 printf("<RIGHT> ");
        rightWheels = generateNextPID(rightWheels);
 printf("speed: %d, %d\n", leftWheels.speed, rightWheels.speed);
    }
}
void moveIn(float inches) {
    moveRaw(inches * MOTOR_COUNTS_PER_INCH);
}
void moveMats(float mats) {
    moveIn(mats * INCHES_PER_MAT);
}

void rotateTo(float targetDeg) {

}
