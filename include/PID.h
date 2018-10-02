#ifndef _PID_H_
#define _PID_H_

typedef struct {
	double Kp;
	double Ki;
	double Kd;
	int error;
	int integral;
	int derivative;
	int target;
	int previousError;
	int *motorPorts;
	int numMotorPorts;
	int startSlowingValue;
} PID_properties_t;

void updatePID(PID_properties_t prop);

PID_properties_t createPID(int *motorPorts, int numMotorPorts, int startSlowingValue);

#include "PID.c"
#endif // _PID_H_