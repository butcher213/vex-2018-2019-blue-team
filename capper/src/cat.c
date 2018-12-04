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

void autonomous();

void getCap();
void flipCap();
void putOnPole();
void putOnBigPole();
void grabCap();
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
// #include "../include/autonomousRed_C.h"
// #include "../include/Sensors_C.h"
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
/***************
*initialize_C.c*
***************/
// #include "../../include/PID.h"
// #include "../include/Sensors_C.h"
// #ifdef _PID_H_
// #warning "INITIALIZE: PID DEFINED"
// #endif

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
**/
void initialize() {
    initializePIDs();

    setupMotor(1, 1, E_MOTOR_GEARSET_18);
    setupMotor(2, 1, E_MOTOR_GEARSET_18);
    setupMotor(11, 0, E_MOTOR_GEARSET_18);
    setupMotor(12, 0, E_MOTOR_GEARSET_18);

    // motor_move(1, 127);
    // motor_move(2, 127);
    // motor_move(11, 127);
    // motor_move(12, 127);

    moveIn(24);
    while (1) {
        printf("DONE");
        delay(100);
    }




	// int leftPorts[] = {11, 12};
	// int rightPorts[] = {1, 2};
	// PID_properties_t left, right;
	// float Kp = .2;
	// float Ki = .00000035;
	// float Kd =  0.0001;
	// left = createPID(Kp, Ki, Kd, leftPorts, 2, 20);
    // right = createPID(Kp, Ki, Kd, rightPorts, 2, 20);
	// left = generateMovedPID(left, 24 * (360 / (PI * 3.6)));
	// right = generateMovedPID(right, 24 * (360 / (PI * 3.6)));
	// while(1) { //&& !atTarget(right)) {
	// 	left = generateNextPID(left);
	// 	right = generateNextPID(right);
	// 	printf("%7.2f\n", left.error);
	// }
	/*
    motor_move(1, 0);
	motor_move(2, 0);
	motor_move(11, 0);
	motor_move(12, 0);
    int a = 1;
    while (a) {
        motor_move(1, 60);
        motor_move(2, 60);
        motor_move(11, 60);
        motor_move(12, 60);
    }

printf("button: %d\n", adi_port_set_config('A', E_ADI_DIGITAL_IN));
printf("setup motor\n");

	int leftPorts[] = {11, 12};
    int rightPorts[] = {1, 2};
	double 	Kp = .8,
			Ki = 0,
			Kd = .55;
    int dist = 20 * (360 / 12.566368);

    int drivePorts[] = {1, 2, 11, 12};
    findKpid_Ziegler(drivePorts, 4, 40, dist);

    // PID_properties_t left, right;
    // for (; !adi_digital_read('A'); Kd += .1) {
    	// left = createPID(Kp, Ki, Kd, leftPorts, 1, 40);
        // right = createPID(Kp, Ki, Kd, rightPorts, 1, 40);
printf("setup PID_properties_t\n");
        // PID_array_t pids = generateRotatedDrive(left, right, dist * 90 / (3.1415 * 15));
        // left = pids[0];
        // right = pids[1];

        // left = generateMovedPID(left, dist);
        // right = generateMovedPID(right, dist);

        // int i;
        // for (; i < 1000 && (!atTarget(left) || !atTarget(right)); i++) {
<<<<<<< HEAD
        for (i = 0; (!atTarget(left) || !atTarget(right)) && i < 200; i++) {
            left = generateNextPID(left);
            right = generateNextPID(right);
printf("%7.2f | %7.2f\n", left.integral, right.integral);*/
// printf("%7.5f | %7.5f | %7.5f\n", Kp, Ki, Kd);
// printf("left   er: %5.2f : %5.2f | i: %12d | d: %12d | t: %5d : %5d\n",
// 		left.error,
//         left.previousError,
// 		left.integral,
//         left.derivative,
//         left.target,
//         motor_get_position(11));
// printf("right  er: %5.2f : %5.2f | i: %12d | d: %12d | t: %5d : %5d\n",
// 		right.error,
//         right.previousError,
// 		right.integral,
//         right.derivative,
//         right.target,
//         motor_get_position(1));
  //      }
    // }
    // PID_properties_t *drive = rotateDrive(left, right, 360);
    // left = drive[0];
    // right = drive[1];
/*
    left = moveTarget(left, 360);
printf("Starting loop\n");
	do {
printf("tick\n");
		left = updatePID(left);
printf("err: %5.2f | itgrl: %5d | drv: %5d\n",
				left.error,
				left.integral,
                left.derivative);
    } while (1 || (int) abs(left.error) > 0);
	motor_move(1, 0);
*/
// printf(">> %-7.2f | %-7.2f\n", left.error, right.error);
printf("END");
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
// #ifdef _PID_H_
// #warning "TELE: PID DEFINED"
// #endif

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
// #warning "In PID source"

PID_properties_t generateNextPID(PID_properties_t prop) {
	int speed, i;

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
// #include "../include/Sensors_C.h"
// #include "../../include/PID.h"
// #ifdef _PID_H_
// #warning "SENSORS: PID DEFINED"
// #endif

PID_properties_t leftWheels, rightWheels;

void initializePIDs() {
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
        leftWheels = generateNextPID(leftWheels);
        rightWheels = generateNextPID(rightWheels);
    }
}
void moveIn(float inches) {
    moveRaw(MOTOR_COUNTS_PER_INCH);
}

void rotateTo(float targetDeg) {

}

void getCap(){

}

void putOnPole() {

}

void putOnBigPole() {

}

void flipCap(){

}

void dropCap(){

}
