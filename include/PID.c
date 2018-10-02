#include "PID.h"
#include "math.h"

int updatePID(PID_properties_t *prop) {
	prop->error = prop->motor_get_position(prop->motorPorts[0]);
	prop->integral += prop->error;

	if (prop->error = 0)
		prop->integral = 0;
	if (abs(prop->error) > prop->startSlowingValue)
		prop->integral = 0;

	prop->derivative = prop->error - prop->previousError;
	prop->previousError = prop->error;

	int speed = prop->Kp * prop->error + prop->Ki * prop->integral + prop->Kd * prop->derivative;

	int i;
	for (i = 0; i < prop->numMotorPorts; ++i)
		motor_move(prop->motorPorts[i], speed);

	return speed;
}

PID_properties_t *createPID(int *motorPorts, int startSlowingValue) {
	PID_properties_t *prop;

	prop->motorPorts = motorPorts;
	prop->startSlowingValue = startSlowingValue;

	return prop;
}
