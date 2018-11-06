#include "PID.h"
#include "main.h"


PID_properties_t generateNextPID(PID_properties_t prop) {
	int speed, i;

    int avgErr = 0;
    for (int i = 0; i < prop.numMotorPorts; i++)
        avgErr += motor_get_position(prop.motorPorts[i]);
    avgErr /= prop.numMotorPorts;

	prop.error = prop.target - avgErr;
	prop.integral += prop.error;

	if (prop.error == 0)
		prop.integral = 0;
	if (abs(prop.error) > prop.startSlowingValue)
		prop.integral = 0;

	prop.derivative = prop.error - prop.previousError;
	prop.previousError = prop.error;

	speed = prop.Kp * prop.error + prop.Ki * prop.integral + prop.Kd * prop.derivative;

    if (speed > 127)
        speed = 127;
    else if (speed < -127)
        speed = -127;

	for (i = 0; i < prop.numMotorPorts; ++i)
		motor_move(prop.motorPorts[i], speed);

    return prop;
}

PID_properties_t generateMovedPID(PID_properties_t prop, double targetDelta) {
    prop.target += targetDelta;
    prop.error += targetDelta;
    return prop;
}

PID_array_t generateRotatedDrive(PID_properties_t left, PID_properties_t right, double target) {
    right = generateMovedPID(right, target);
    left = generateMovedPID(left, -target);

    do {
        right = generateNextPID(right);
        left = generateNextPID(left);
    } while (abs(right.error) > 0 || abs(left.error) > 0);

    PID_properties_t *drive = malloc(sizeof(PID_properties_t) * 2);
    drive[0] = left;
    drive[1] = right;
    return drive;
}

int atTarget(PID_properties_t prop) {
    if (abs(prop.error) < 5)
        return 1;
    else
        return 0;
}

PID_properties_t createPID(double Kp, double Ki, double Kd, int *motorPorts, int numMotorPorts, int startSlowingValue) {
	PID_properties_t prop;

	prop.Kp = Kp;
	prop.Ki = Ki;
	prop.Kd = Kd;
	prop.numMotorPorts = numMotorPorts;
	prop.motorPorts = motorPorts;
	prop.startSlowingValue = startSlowingValue;

    prop.target = 0;
    prop.error = 0;
    prop.previousError = 0;
    prop.derivative = 0;
    prop.integral = 0;

	return prop;
}

PID_properties_t applyRealTimeCorrection(PID_properties_t prop) {
    if (prop.derivative == 0) {
        if (prop.error < 0) { // robot went too far; derivative is too high
            prop.Kd -= .005;
        }
        else if (prop.error > 0) { // robot did not go far enough; derivative is too low
            prop.Kd += .005;
        }
    }

    return prop;
}

PID_properties_t findKpid(int* motorPorts, int numMotorPorts, int startSlowingValue, int target) {
    PID_properties_t prop = createPID(0, 0, 0, motorPorts, numMotorPorts, startSlowingValue);

    int runSimulation = 1;
    int constToChange = 0;

    for (int n = 0; runSimulation; n++) {
        prop.target = n%2 * target; // swap between driving forward and backward

        switch (constToChange) {
            case 0:
                prop.Kp += .05;
                break;
            case 1:

                break;
            case 2:
                break;
        }

        for (int i = 0; i < 1000; i++)
            prop = generateNextPID(prop);

        for (int port: motorPorts)

    }
}
