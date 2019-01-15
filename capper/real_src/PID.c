#include "../include/PID.h"
#include "../include/main_C.h"

PID_properties_t generateNextPID(PID_properties_t prop) {
    prop.error = calculateError(prop);
 /* =================================================
  * = Replaced with calculateError(), needs testing =
  * =================================================

 printf("1: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
    int avgPosition = 0;
    for (int i = 0; i < prop.numMotorPorts; i++)
        avgPosition += motor_get_position(prop.motorPorts[i]);
    avgPosition /= prop.numMotorPorts;

 printf("2: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
 	prop.error = prop.target - avgPosition;
 */

 printf("3: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
	if (abs(prop.error) <= prop.startSlowingValue)
		prop.integral = 0;
    else
    	prop.integral += prop.error;

 printf("4: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
	prop.derivative = prop.error - prop.previousError;
	prop.previousError = prop.error;

 printf("5: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
	prop.speed = prop.Kp * prop.error + prop.Ki * prop.integral + prop.Kd * prop.derivative;

 printf("6: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
    if (prop.speed > 127)
        prop.speed = 127;
    else if (prop.speed < -127)
        prop.speed = -127;

 printf("7: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
    // printf("motor ports: ");
	for (int i = 0; i < prop.numMotorPorts; i++) {
		motor_move(prop.motorPorts[i], prop.speed);
        // printf("%d(%d), ", prop.motorPorts[i], i);
    }
    // printf("\n");

 printf("8: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
    return prop;
}

int calculateError(PID_properties_t prop) {
 printf("1: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);

    int avgPosition = 0;
    for (int i = 0; i < prop.numMotorPorts; i++)
        avgPosition += motor_get_position(prop.motorPorts[i]);
    avgPosition /= prop.numMotorPorts;

 printf("2: %d, %d", prop.motorPorts[0], prop.motorPorts[1]);
	return prop.target - avgPosition;
}

#warning "Untested function: generateNextPID()"
PID_array_t generateNextSSPID(PID_array_t pids, int length) {
    // calculate errors of each PID
    int totalErrors[length];
    for (int i = 0; i < length; i++)
        totalErrors[i] = calculateError(pids[i]);

    // calulate each PID's distance sum from other PIDs
    for (int i = 0; i < length; i++) {
        // add on errors from other PIDs in system
        for (int j = 0; j < length; j++) {
            if (i == j) // no need to calculate for current PID
                continue;

            int dist = pids[i].error - pids[j].error;
            // add dist to each other PID in error
            totalErrors[i] += dist;
        }
    }

    // store total errors back into their respective PID
    for (int i = 0; i < length; i++)
        pids[i].error = totalErrors[i];

    // calculate PID speed normally from here
 #warning "[PID.c ~92] SSPID not implemented"
}

PID_properties_t generateMovedPID(PID_properties_t prop, long long targetDelta) {
    prop.target += targetDelta;
    prop.error += targetDelta;
    return prop;
}

PID_array_t generateRotatedDrive(PID_properties_t left, PID_properties_t right, long long target) {
    left = generateMovedPID(left, -target);
    right = generateMovedPID(right, target);

    do {
        left = generateNextPID(left);
        right = generateNextPID(right);
    } while (!atTarget(left) || !atTarget(right));

    PID_array_t drive = malloc(sizeof(PID_properties_t) * 2);
    drive[0] = left;
    drive[1] = right;
    return drive;
}

int atTarget(PID_properties_t prop) {
    return isStopped(prop) && abs(prop.error) < 5;
}

int isStopped(PID_properties_t prop) {
    return prop.derivative == 0;
}

PID_properties_t createPID(double Kp, double Ki, double Kd, int *motorPorts, int numMotorPorts, long long startSlowingValue) {
	PID_properties_t prop;

	prop.Kp = Kp;
	prop.Ki = Ki;
	prop.Kd = Kd;

    prop.speed = 0;
    prop.error = 0;
    prop.integral = 0;
    prop.derivative = 0;
    prop.target = 0;
    prop.previousError = 0;

	prop.motorPorts = motorPorts;
	prop.numMotorPorts = numMotorPorts;
	prop.startSlowingValue = startSlowingValue;

    // printf("ports: %d, %d\n", prop.motorPorts[0], prop.motorPorts[1]);
	return prop;
}
