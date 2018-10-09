#include "PID.h"
#include "main.h"

PID_properties_t updatePID(PID_properties_t prop) {
	int speed, i;

	prop.error = prop.target - motor_get_position(prop.motorPorts[0]);
	prop.integral += prop.error;

	if (prop.error == 0)
		prop.integral = 0;
	if (abs(prop.error) > prop.startSlowingValue)
		prop.integral = 0;

	prop.derivative = prop.error - prop.previousError;
	prop.previousError = prop.error;

	speed = prop.Kp * prop.error + prop.Ki * prop.integral + prop.Kd * prop.derivative;

	for (i = 0; i < prop.numMotorPorts; ++i)
		motor_move(prop.motorPorts[i], speed);
}

PID_properties_t createPID(double Kp, double Ki, double Kd, int *motorPorts, int numMotorPorts, int startSlowingValue) {
	PID_properties_t prop;

	prop.Kp = Kp;
	prop.Ki = Ki;
	prop.Kd = Kd;
	prop.numMotorPorts = numMotorPorts;
	prop.motorPorts = motorPorts;
	prop.startSlowingValue = startSlowingValue;

	return prop;
}
