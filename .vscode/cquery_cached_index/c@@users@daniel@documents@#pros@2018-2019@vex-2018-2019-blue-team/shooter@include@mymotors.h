#ifndef _MYMOTORS_H_
#define _SENSORS_H_

#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2
#define WHEELS_FORWARD 127
#define WHEELS_BACKWARD -127
#define WHEEL_DIAMETER 4
/* Function:		leftWheels
 * Purpose:			moves the left side of the robot forward
 * Arguments:	  speed: the speed of the motors
 * Returns:			N/A
 */
void myleftWheels(float speed);

/* Function:		rightWheels
 * Purpose:			moves the right side of the robot forward
 * Arguments:	  speed: the speed of the motors
 * Returns:			N/A
 */
void myrightWheels(float speed);
#include "Mymotors.c"
#endif // _MYMOTORS_H_
