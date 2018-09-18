#ifndef _MOTORS_H_
#define _MOTORS_H_

#include "Controller_Template.h"

// wheel constants
#define DRIVE_FORWARD		   127
#define DRIVE_BACKWARD		-DRIVE_FORWARD
#define STRAFE_LEFT			   127
#define STRAFE_RIGHT		  -STRAFE_LEFT
// arm constants
#define CONE_ARM_DOWN			-127
#define CONE_ARM_UP				-CONE_ARM_DOWN
// claw constants
#define CONE_CLAW_CLOSE		 127
#define CONE_CLAW_OPEN		-CONE_CLAW_CLOSE
// mobile goal lift constants
#define GOAL_ARM_UP				 127
#define GOAL_ARM_DOWN			-GOAL_ARM_UP
// mobile goal claw constants
#define GOAL_CLAW_OPEN		 127
#define GOAL_CLAW_CLOSE		-GOAL_CLAW_OPEN
// mobile goal pusher constants
#define GOAL_PUSHER_OPEN   127
#define GOAL_PUSHER_CLOSE -GOAL_PUSHER_OPEN
// cone arm potentiometer positions
#define CONE_ARM_GROUND		 0
#define CONE_ARM_FEEDER		 280
#define CONE_ARM_GOAL			 500 // TODO
#define CONE_ARM_HIGH			 726
#define CONE_ARM_BACK			 1400


void rightWheels(int speed);
void leftWheels(int speed);
void strafeWheel(int speed);
void driveStraight(int speed);
void coneArmSpeed(int speed);
void coneClawSpeed(int speed);
void goalArmSpeed(int speed);
void goalClawSpeed(int speed);
void goalPusherSpeed(int speed);


#include "Motors.c"

#endif // _MOTORS_H_
