#ifndef _PID_H_
#define _PID_H_

typedef struct {
	double Kp;
	double Ki;
	double Kd;
	double error;
	int integral;
	int derivative;
	double target;
	double previousError;
	int *motorPorts;
	int numMotorPorts;
	int startSlowingValue;
} PID_properties_t;

/* Function:		updatePID
 * Purpose:			Updates the PID_properties_t by running through one pass of the PID algorithm
 * Argument:		prop = the property to be updated
 * Return:			N/A
 */
PID_properties_t updatePID(PID_properties_t prop);

/* Function:		createPID
 * Purpose:			Generates a new PID_properties_t object using the parameters
 * Argument:		motorPorts = the motor ports that are associated with the PID_properties_t
 					numMotorPorts = the length of motorPorts
					startSlowingValue = the error value where the motors will start to slow down
 * Return:			The created PID_properties_t object
 */
PID_properties_t createPID(double Kp, double Ki, double Kd, int *motorPorts, int numMotorPorts, int startSlowingValue);

#include "PID.c"
#endif // _PID_H_
