#ifndef _SENSORS_H_
#define _SENSORS_H_

#define WHEEL_DIAMETER 3
#define PI 3.1415
#define Pole_Hight_Small 23.0
#define Pole_Hight_Large 34.0 
#define MAT_Size 22.1

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


#include "../src/Sensors_C.c"
#endif // _SENSORS_H_
