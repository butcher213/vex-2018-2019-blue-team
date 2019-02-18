/********
*api_S.h*
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
/*********
*main_S.h*
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

// long int pid_target = 0;
// long int pid_speed = 0;

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
/*************
*Mymotors_S.h*
*************/
#ifndef _MYMOTORS_H_
#define _MYMOTORS_H_

#define MOTOR_FRONT_LEFT 11
#define MOTOR_FRONT_RIGHT 3
#define MOTOR_BACK_LEFT 2
#define MOTOR_BACK_RIGHT 13
#define MOTOR_CATAPULT_LEFT 5
#define MOTOR_CATAPULT_RIGHT 6
#define MOTOR_INTAKE 7
#define MOTOR_FLAPPER 8
#define MOTOR_BELT 12
#define WHEELS_FORWARD 127
#define WHEELS_BACKWARD -127
#define WHEEL_DIAMETER 4
#define PI 3.1415
#define TILE_LENGTH 22.5



//void initMotors(int motor, int gearset, bool reversed);
//PID_array_t initDrive(double Kp, double Ki, double Kd);

void initPID();

void moveIn(double left, double right);

void turnDeg(double deg);

void launchCatapult(void);

void stopDriveMotors(void);

void spinIntake(double multiplier);
#endif // _MYMOTORS_H_
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
*Sensors_S.h*
************/
#ifndef _SENSORS_S_H_
#define _SENSORS_S_H_

/* Function:		rotateTo
 * Purpose:			rotates the robot to the specified degree
 * Argument:		deg = degree to move to
 * Return:			N/A
 */
void rotateTo(float targetDeg);

void PID_control();

#endif // _SENSORS_S_H_
/***************
*autonomous_S.c*
***************/
#define MAT_Size 22.1

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
/*void autonomous() {
  // 1 for red, 0 for blue, anything else for no auton
int color = 1;
  if(color == 1) {
    // Launches preload ball and fed ball into the top targets
    spinIntake(1);
    delay(5000);
    spinIntake(0);
    loadBallsIntoCatapult();
    moveIn(5, 5);
    stopDriveMotors();
    delay(1000);
    launchCatapult();
    delay(1000);
    // push the lower flag
    moveIn(TILE_LENGTH *.9, TILE_LENGTH*.9);
    // ------------------------ blue auton -------------------------------------

  } else {

<<<<<<< HEAD
  }
}*/
=======
  }*/
//}
>>>>>>> 98da6422f82f19338661a4e408db8a5ce2d88f2b
/***************
*initialize_S.c*
***************/
//#include "../../include/PID.h"

void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *``
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  initMotors(MOTOR_FRONT_LEFT, E_MOTOR_GEARSET_18, 0);
  initMotors(MOTOR_FRONT_RIGHT, E_MOTOR_GEARSET_36, 1);
  initMotors(MOTOR_BACK_RIGHT, E_MOTOR_GEARSET_36, 1);
  initMotors(MOTOR_BACK_LEFT, E_MOTOR_GEARSET_18, 0);
  initMotors(MOTOR_CATAPULT_LEFT, E_MOTOR_GEARSET_18, 0);
  initMotors(MOTOR_CATAPULT_RIGHT, E_MOTOR_GEARSET_18, 1);
  initMotors(MOTOR_BELT, E_MOTOR_GEARSET_18, 0);
  initMotors(MOTOR_INTAKE, E_MOTOR_GEARSET_18, 0);
  initPID();
  int drivingVar = 1;
<<<<<<< HEAD

  moveIn(24, 24);
  while(1);

=======
>>>>>>> 98da6422f82f19338661a4e408db8a5ce2d88f2b
  int color = 1;
  // ------------------------ red auton --------------------------------------
  if(color == 1) {
    // Launches preload ball and fed ball into the top targets
    /*spinIntake(1);
    delay(5000);
    spinIntake(0);*/
    loadBallsIntoCatapult();
    moveIn(5, 5);
    stopDriveMotors();
    delay(1000);
    launchCatapult();
    delay(1000);
    // push the lower flag
    moveIn(TILE_LENGTH *.9, TILE_LENGTH*.9);
  }
}

  /* Move Inches Prototype */

  /*int left[2] = {MOTOR_FRONT_LEFT, MOTOR_BACK_LEFT};
  int right[2] = {MOTOR_FRONT_RIGHT, MOTOR_BACK_RIGHT};


  double kp = 5, ki = 0, kd = 0;

  PID_properties_t PIDs[2] = {createPID(kp,ki,kd, left, 2, 40), createPID(kp,ki,kd, right, 2, 40)};

  double leftDist = 12;
  double rightDist = 12;
  printf("go\n");
  PID_properties_t a[2] = {generateMovedPID(PIDs[0], 360/(4*PI)*leftDist), generateMovedPID(PIDs[1], 360/(4*PI)*rightDist)};
    while (true) {
      printf("%d, %f\n", atTarget(a[0]), a[0].error);
      a[0] = generateNextPID(a[0]);
      a[1] = generateNextPID(a[1]);
    }
    /*int motorPorts[] = {1,2,3,4};
    int numMotorPorts = 4;
    int startSlowingValue = 40;
    int target = 360;
    PID_properties_t ziegler = findKpid_Ziegler(motorPorts, numMotorPorts, startSlowingValue, target);*/


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
//void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
//void competition_initialize() {}
/*************
*Mymotors_S.c*
*************/


/* Function:		initMotors
 * Purpose:			Get Motors Initialized
 * Arguments:	  motor: motor ports, gearset: gearset, reversed, should the motor be reversed
 * Returns:			N/A
 */
void initMotors(int motor, int gearset, bool reversed) {
   motor_set_gearing(motor, gearset);
   motor_set_reversed(motor, reversed);
   motor_set_encoder_units(motor, E_MOTOR_ENCODER_DEGREES);
  // motor_tare_position(motor);
 }


/* Function:		initDrive
 * Purpose:			Get PID's Made
 * Arguments:	  Kp: P, Ki: I, Kd: D
 * Returns:			{PID_properties_t left, PID_properties_t right}
 */
// PID_array_t initDrive(double Kp, double Ki, double Kd) {
//   printf("initializing\n");
//   initMotors(MOTOR_FRONT_LEFT, E_MOTOR_GEARSET_18, 0);
//   initMotors(MOTOR_FRONT_RIGHT, E_MOTOR_GEARSET_18, 0);
//   initMotors(MOTOR_BACK_LEFT, E_MOTOR_GEARSET_18, 1);
//   initMotors(MOTOR_BACK_RIGHT, E_MOTOR_GEARSET_18, 1);
//   int left[2] = {MOTOR_FRONT_LEFT, MOTOR_BACK_LEFT};
//   int right[2] = {MOTOR_FRONT_RIGHT, MOTOR_BACK_RIGHT};
//   PID_properties_t a[2] = {createPID(Kp, Ki, Kd, left, 2, 40), createPID(Kp, Ki, Kd, right, 2, 40)};
//   return a;
// }



/* Function:		initDrive
 * Purpose:			Get PID's Made
 * Arguments:	  PIDs: PID_properties_t for left and right, left: left target, right: right target
 * Returns:			{PID_properties_t left, PID_properties_t right}
 */

PID_properties_t rightMotors, leftMotors;

void initPID() {
  static int rightMotorPorts[] = {MOTOR_BACK_RIGHT, MOTOR_FRONT_RIGHT};
  rightMotors = createPID(0.5, 0.000009, 0.009, rightMotorPorts, 2, 40);
  static int leftMotorPorts[] = {MOTOR_BACK_LEFT, MOTOR_FRONT_LEFT};
  leftMotors = createPID(0.5, 0.000009, 0.009, leftMotorPorts, 2, 40);
}


void moveIn(double left, double right) {
  PID_properties_t a[2] = {generateMovedPID(leftMotors, 360/(4*PI)*left), generateMovedPID(rightMotors, 360/(4*PI)*right)};

  printf("%d\n", atTarget(a[0]));
  bool flag = 0;
  //a[0].error = a[1].error;
  leftMotors = a[0];
  rightMotors = a[1];

  while (!atTarget(a[0]) && !atTarget(a[1])) {
    a[0] = generateNextPID(a[0]);
    a[1] = generateNextPID(a[1]);
  //  printf("Left: %d       Right: %d\n", atTarget(a[1]), atTarget(a[0]));
  }
  leftMotors = a[0];
  rightMotors = a[1];
  motor_move(MOTOR_FRONT_LEFT, 0);
  motor_move(MOTOR_FRONT_RIGHT, 0);
  motor_move(MOTOR_BACK_LEFT, 0);
  motor_move(MOTOR_BACK_RIGHT, 0);
}


void turnDeg(double deg) {
  double dist = deg * 180/PI * 16/2;

  moveIn(dist, -dist);

}
/* Function:		stopDriveMotors
 * Purpose:			stops all drive motors.
 * Arguments:	  N/A
 * Returns:			N/A
 */

void stopDriveMotors(void) {
  motor_move(MOTOR_BACK_LEFT, 0);
  motor_move(MOTOR_FRONT_LEFT, 0);
  motor_move(MOTOR_BACK_RIGHT, 0);
  motor_move(MOTOR_FRONT_RIGHT, 0);
}


/* Function:		launchCatapult
 * Purpose:		  Launch the catapult arm to throw a ball in autonomous
 * Arguments:	  N/A
 * Returns:			N/A
 */

void launchCatapult(void) {
  stopDriveMotors();
  printf("voltage : %d\n", motor_get_current_draw(MOTOR_CATAPULT_LEFT));
  int max = 0;
  while(motor_get_current_draw(MOTOR_CATAPULT_LEFT) < 1900) {
    if(motor_get_current_draw(MOTOR_CATAPULT_LEFT) > max ) {
    printf("max: %d\n", motor_get_current_draw(MOTOR_CATAPULT_LEFT));
    max = motor_get_current_draw(MOTOR_CATAPULT_LEFT);
  }
    motor_move(MOTOR_CATAPULT_LEFT, 127);
    motor_move(MOTOR_CATAPULT_RIGHT, 127);
  }
  //delay(50);
  motor_move(MOTOR_CATAPULT_LEFT, 0);
  motor_move(MOTOR_CATAPULT_RIGHT, 0);
}
/* Function:		spinLoader
 * Purpose:		  spins the loading mechanism to load a ball into the flapper.
 *              No exit condition
 * Arguments:	  multiplier - a number between -1 and 1 to spin the motors at.
  *             .75 for 75% and so on.
 * Returns:			N/A
 */

void spinIntake(double multiplier) {
motor_move(MOTOR_INTAKE, 127 * multiplier);
motor_move(MOTOR_BELT, 127 * multiplier);
}

/* Function:		loadBallsIntoCatapult
 * Purpose:		  dumps balls from the flapper wrist into the catapult
 * Arguments:	  N/A
 * Returns:			N/A
 */

void loadBallsIntoCatapult(void) {
  while(adi_digital_read('G') == 0) {
    motor_move(MOTOR_CATAPULT_LEFT, 127);
    motor_move(MOTOR_CATAPULT_RIGHT, 127);
  }
  motor_move_velocity(MOTOR_CATAPULT_LEFT, 1);
  motor_move_velocity(MOTOR_CATAPULT_RIGHT, 1);
  delay(100);
  // dump balls
  motor_move(MOTOR_FLAPPER, -25);
  delay(500);
  motor_move(MOTOR_FLAPPER,0);
  delay(500);
  motor_move(MOTOR_FLAPPER, 25);
  delay(500);
  motor_move(MOTOR_FLAPPER,0);
  motor_move(MOTOR_CATAPULT_LEFT, 0);
  motor_move(MOTOR_CATAPULT_RIGHT, 0);
}
/**************
*opcontrol_S.c*
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

 /* Current Driving Code 1/15/2019*/
void opcontrol() {
  double flapperPos;
  int drivingVar = 1;
  while(1){

/*  R2 - Shoot
    B - Manual Shoot Cancel
    L2(Hold) - Slow Drive
    L1(Hold) - Intake
    RIGHT - Dump Flapper

    Tank Drive:
    Left stick control left wheels, right stick controls right wheels.

    Arcade Drive:
    Left stick drives forward, backward, left, and right. Does Everything.

    Modified Arcade Drive:
    If you've played Halo 3 think of it like that. Left stick forward drives forward and backwards, right stick turns left and right.
 */






    /* Tank Drive */

    /* R2 Holding = Slow Drive */
    /*if((controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1) && drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y) / 4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y) / 4);
    }*/

    /* Quick Drive, Slow Hold Disabled */

    /*else if(drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y));
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y));
    }*/

    /* Modified Arcade Drive */

    /* R2 Holding = Slow Drive */
    if((controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1) && drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / -4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / -4);
    }

    /* Quick Drive, Slow Hold Disabled */

    else if(drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) * -1);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) * -1);
    }

    /* Classic Arcade Drive */

    /* R2 Holding = Slow Drive */
    /*if((controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1) && drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) / -4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) / -4);
    }*/

    /* Quick Drive, Slow Hold Disabled */
    /*else if(drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) * -1);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) * -1);
    }*/


    /* Digital Buttons */

    /* Shoot */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R2) == 1){
      motor_move(MOTOR_CATAPULT_LEFT, 127);
      motor_move(MOTOR_CATAPULT_RIGHT, 127);
      motor_move(MOTOR_BACK_LEFT, 0);
      motor_move(MOTOR_FRONT_LEFT, 0);
      motor_move(MOTOR_BACK_RIGHT, 0);
      motor_move(MOTOR_FRONT_RIGHT, 0);
      drivingVar = 0;
    }
    /* Sensor Triggered Cancel Shooting */
    if(motor_get_current_draw(MOTOR_CATAPULT_LEFT) > 1550){
      delay(50);
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);
      drivingVar = 1;
    }

    /* Cancell Shooting */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_B) == 1){
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);
      drivingVar = 1;
    }

    /* Intake Hold */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L1) == 1){
      motor_move(MOTOR_INTAKE, 127);
      motor_move(MOTOR_BELT, 127);
    }
    else{
      motor_move(MOTOR_INTAKE, 0);
      motor_move(MOTOR_BELT, 0);
    }

    /* Flapper - Dump */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_RIGHT) == 1)
    {
      while(adi_digital_read('G') == 0) {
        motor_move(MOTOR_CATAPULT_LEFT, 127);
        motor_move(MOTOR_CATAPULT_RIGHT, 127);
      }
      motor_move_velocity(MOTOR_CATAPULT_LEFT, 1);
      motor_move_velocity(MOTOR_CATAPULT_RIGHT, 1);
      delay(100);
      // dump balls
      motor_move_absolute(MOTOR_FLAPPER, 200, 60);
      delay(500);
      motor_move_absolute(MOTOR_FLAPPER, -20, 75);
      delay(500);
      motor_move(MOTOR_FLAPPER,0);
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);

    }


  }
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

    if (speed > 75)
        speed = 75;
    else if (speed < -75)
        speed = -75;

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
    return isStopped(prop) && abs(prop.error) < 10;
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
*Sensors_S.c*
************/
#ifndef PI
#define PI 3.1415
#endif
/* Function:		moveIn
 * Purpose:			moves the robot a specified amount of inches
 * Argument:		inches = amount of inches to move
 * Returns:			N/A
 */

/* Function:		rotateTo
 * Purpose:			rotates the robot to the specified degree
 * Argument:		deg = degree to move to
 * Return:			N/A
 */
 void rotateTo(float targetDeg) {
    float Wheel_Spread = 14.625;
    float Max_Speed = 63;
    motor_move_relative(1, ((Wheel_Spread*PI*targetDeg)/(4*PI)), Max_Speed);
    motor_move_relative(2, -((Wheel_Spread*PI*targetDeg)/(4*PI)), Max_Speed);
  }

/* Function:		PID_control
 * Purpose:     ???
 * Argument     N/A
 * Return       N/A
 */

void PID_control() {
 /* float K_p = .5;
  float K_i = 0;
  float K_d = 0;
  motor_set_zero_position(2,0);
  motor_set_zero_position(1,0);
  float K_p = .5;
  float K_i = 0.000009;
  float K_d = 0.009;
  double integral_left = 0;
  double integral_right = 0;
  float prev_error_left = 0;
  float prev_error_right = 0;
  float pid_target = (40 * (360 / (PI * 2)));
  while(1) {
    double error_left = pid_target - motor_get_position(1);
    printf("error left %.2f\n", error_left);
    double error_right = pid_target - motor_get_position(2);
    printf("error right %.2f\n", error_right);
    integral_left += error_left;
    integral_right += error_right;
    if (error_left == 0 || abs(error_left) > 40){
      integral_left = 0;
    }
    if (error_right == 0 || abs(error_right) > 40){
      integral_right = 0;
    }
    double deriv_left = error_left - prev_error_left;
    prev_error_left = error_left;
    float pid_speed_left = K_p * error_left + (K_i * integral_left + K_d * deriv_left);
    printf("%f left speed\n", pid_speed_left);
    double deriv_right = error_right - prev_error_right;
    prev_error_right = error_right;
    float pid_speed_right = K_p * error_right + (K_i * integral_right + K_d * deriv_right);
    printf("%f right speed\n", pid_speed_right);
    pid_speed_right = K_p * error_right + K_i * integral_right + K_d * deriv_right;
    motor_move(1,pid_speed_left);
    motor_move(1,pid_speed_right);
    motor_move(2,pid_speed_right);
  }
  */
}
