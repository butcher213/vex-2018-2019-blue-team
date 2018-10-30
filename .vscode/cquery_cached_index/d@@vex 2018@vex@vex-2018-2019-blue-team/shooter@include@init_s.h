#ifndef _INIT_H_
#define _INIT_H_

/* Function:		initMotor
 * Purpose:			Creates a new motor in PROS
 * Arguments:		motorPort: the port of the motor
 *              direction: 0 if normal, 1 if reversed
 * Returns:			N/A
 */
void initMotor(int motorPort, int direction);


#include "Init_S.c"
//#include "Init.c"
#endif // _INIT_H_
