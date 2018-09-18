#include "Init.h"

void doPreAuton() {
  bStopTasksBetweenModes = true;
  resetGyro(Gyro);
  resetEncoder(ArmEncoder);
}

void resetEncoder(int port) {
  SensorValue[port];
}

void resetGyro(int port) {
  SensorType[port] = sensorNone;
	wait10Msec(100);
	SensorType[port] = sensorGyro;
	wait10Msec(100);
}
