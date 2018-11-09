#include "PID.h"
#include "main.h"


PID_properties_t generateNextPID(PID_properties_t prop) {
	int speed, i;

    int avgPosition = 0;
    for (int i = 0; i < prop.numMotorPorts; i++)
        avgPosition += motor_get_position(prop.motorPorts[i]);
    avgPosition /= prop.numMotorPorts;

	prop.error = prop.target - avgPosition;
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
    if (prop.derivative == 0 && abs(prop.error) < 5)
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
            prop.Ki -= .0000000001;
        }
        else if (prop.error > 0) { // robot did not go far enough; derivative is too low
            prop.Ki += .0000000001;
        }
    }

    return prop;
}

PID_properties_t findKpid_Ziegler(int* motorPorts, int numMotorPorts, int startSlowingValue, int target) {
    /* part 1 of PDF -
     *  Set Kp, Ki, Kd to 0. This will disable them for now.
    */
    PID_properties_t prop = createPID(0, 0, 0, motorPorts, numMotorPorts, startSlowingValue);

    int dir = 1;

    while (1) {
        prop.target = target * dir;
        dir = 1 - dir;

        prop.Kp += .05;

        int lastDir = 2 * dir - 1;
        int lastPeriodMeasured; // first measurement will be bad
        do {
            prop = generateNextPID(prop);

            // is this pass the start of a new period measurement?
            int currentDir = (prop.derivative > 1)? 1: -1;

            // if (changed direction of motion && direction starts new period)
            if (currentDir == -lastDir && currentDir == 2 * dir - 1) {
                // set priod to be new measured value
                int now = millis();
                int period = now - lastPeriodMeasured;
                lastPeriodMeasured = now;

                // print new period to console
                printf("Pu=%6d | Ku=%5.2f\n", period, prop.Kp);
            }

            lastDir = currentDir;
        } while (!atTarget(prop));

        delay(3000); // wait for robot to settle
    }
}

PID_properties_t findKpid_manual(int* motorPorts, int numMotorPorts, int startSlowingValue, int target) {
    /* part 1 of PDF -
     *  Set Kp, Ki, Kd to 0. This will disable them for now.
    */
    PID_properties_t prop = createPID(0, 0, 0, motorPorts, numMotorPorts, startSlowingValue);

    int dir = 1; // 1 for forward, 0 for return to starts. switch w/ dir = 1-dir;

    // the Kp, Ki, Kd tuning constants, acceptable errors
    double tuningVars[3][2] = { {.5,            20}, // for Kp
                                {.002,          10}, // for Kd
                                {.0000000001,   5}}; // for Ki

    for (int i = 0; i < 3; i++) {
        do {
            prop.target = target * dir;
            prop.error = target; // error must be at max to begin with
            dir = 1 - dir;

            switch (i) {
                case 0:
                    prop.Kp += tuningVars[0][0];
                    break;
                case 1:
                    prop.Kd += tuningVars[1][0];
                    break;
                case 2:
                    prop.Ki += tuningVars[2][0];
                    break;
            }

            for (int j = 0; j < 1000 && abs(prop.error) > tuningVars[i][1]; j++)
                prop = generateNextPID(prop);

            delay(3000); // wait for robot to settle
        } while (abs(prop.error) > tuningVars[i][1]);
    }

    // print out values
    while (1)
        printf("Kpid=%5.2f | %5.2f | %5.2f\n", prop.Kp, prop.Ki, prop.Kd);
}
