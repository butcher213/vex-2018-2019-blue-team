#ifndef _SENSORS_H_
#define _SENSORS_H_

// the diameter of the robot's drive train wheels
#define WHEEL_DIAMETER				4.125
// the calculated circumfrance of WHEEL_DIAMETER
#define WHEEL_CIRCUMFERENCE		(PI * WHEEL_DIAMETER)
// maximum value of potentiometer
#define POT_MAX								4095.0
// maximum value returned by getGyro()
#define GYRO_MAX							3600.0
// change of value of encoder after one full revolution
#define ENCODER_VALUE_PER_REV	360.0
#define FEEDER_CONE_POS 941
#define FEEDER_POS 1141
#define GROUND_POS 1400
#define STACK_POS 300
#define STACK_REL_POS 500

/* Function:		getGyro
 * Purpose:			retrieves the value of the robot's gyroscope
 * Argument:		N/A
 * Returns:			the value of the robot's gyro from 0 - GYRO_MAX
 */
int getGyro(void);

/* Function:		getArmPot
 * Purpose:			retrieves the value of the robot's claw arm potentiometer
 * Argument:		N/A
 * Returns:			the value of the robot's arm potentiometer from 0 - POT_MAX
 */
int getArmPot(void);

/* Function:		moveIn
 * Purpose:			moves the robot a specified amount of inches
 * Argument:		inches = amount of inches to move
 * Returns:			N/A
 */
void moveIn(float inches);

/* Function:		strafeIn
 * Purpose:			moves the robot side-to-side for a specified amount of inches
 * Argument:		inches = amount of inches to move
 * Return:			N/A
 */
void strafeIn(float inches);

/* Function:		rotateTo
 * Purpose:			rotates the robot to the specified degree
 * Argument:		deg = degree to move to
 * Return:			N/A
 */
void rotateTo(float targetDeg);

/* Function:		getArmPosition
 * Purpose:			moves the claw arm to a position based off the arm potentiometer
 * Argument:		position	= the arm potentiometer value to aim for
 * Return:			N/A
*/
int getArmPosition();

int getGoalLiftBump();



#include "Sensors.c"

#endif //_SENSORS_H_
