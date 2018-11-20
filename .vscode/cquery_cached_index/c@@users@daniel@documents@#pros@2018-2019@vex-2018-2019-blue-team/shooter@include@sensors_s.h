#ifndef _SENSORS_S_H_
#define _SENSORS_S_H_

/* Function:		rotateTo
 * Purpose:			rotates the robot to the specified degree
 * Argument:		deg = degree to move to
 * Return:			N/A
 */
void rotateTo(float targetDeg);

void PID_control();

#include "Sensors_S.c"
#endif // _SENSORS_S_H_
