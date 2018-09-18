#ifndef _AUTON_H_
#define _AUTON_H_

//defs
#define RED					1
#define BLUE				0
#define LEFT_TURN		1
#define RIGHT_TURN	2

/* Function: 		doAuton
 * Purpose: 		called in main.c
 * Arguements:	N/A
 * Returns: 		N/A
*/
void doAuton();

void blueAuton();

void redAuton();

/* Function: 		turnForDegrees
 * Purpose: 		turn the robot for a specifed degree and a specified angle
 * Arguements:
 * Returns: 		nothing
*/
void turnForDegrees(double angle);

/* Function: 		getStartPosition
 * Purpose: 		retrieves the input for the starting position (RED/ BLUE)
 * Arguements:	N/A
 * Returns: 		the value of dgtl1
*/
int getStartPosition();

/* Function: 		retrieveMobile
 * Purpose: 		moves from the start and grabs the mobile goal
 * Arguements:	N/A
 * Returns: 		N/A
*/
void retrieveMobile();
void retrieveMobileRed();
/* Function: 		stackConeFromGround
 * Purpose: 		grabs a cone on the ground and stacks it
 * Arguements:	N/A
 * Returns: 		N/A
*/
void stackConeFromGround();

/* Function: 		moveToFeederStation
 * Purpose: 		moves the robot to the feeder station from the first mbile goal
 * Arguements:	N/A
 * Returns: 		N/A
*/
void moveToFeederStation();
void moveToFeederStationRed();
/* Function: 		stackConeFromFeeder
 * Purpose: 		grabs a cone from the feeder and stacks it
 * Arguements:	N/A
 * Returns: 		N/A
*/
void setArmToFeeder();
void stackConeFromFeeder();

/* Function: 		depositFirstStack
 * Purpose: 		moves the robot to the start from the feeder station
 *                and drops its current stack in the 20pt zone
 * Arguements:	N/A
 * Returns: 		N/A
*/
void depositFirstStack();

/* Function:
 * Purpose:
 * Arguements:
 * Returns:
*/
void lowerBase();

/* Function:
 * Purpose:
 * Arguements:
 * Returns:
*/
void raiseBase();

void placeBase();
#include "Auton.c"

#endif // _AUTON_H_
