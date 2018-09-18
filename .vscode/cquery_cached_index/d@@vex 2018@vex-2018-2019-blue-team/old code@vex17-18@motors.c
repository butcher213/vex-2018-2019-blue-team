
void rightWheels(int speed) {
	motor[WheelsRight] = speed;
}

void leftWheels(int speed) {
	motor[WheelsLeft] = speed;
}

void strafeWheel(int speed) {
	motor[WheelStrafe] = speed;
}

void driveStraight(int speed) {
	rightWheels(speed);
	leftWheels(speed);
}

void coneArmSpeed(int speed) {
	motor[ConeArm] = speed;
}

void coneClawSpeed(int speed) {
	motor[ConeClaw] = speed;
}

void goalArmSpeed(int speed) {
	motor[GoalArm] = speed;
}

void goalPusherSpeed(int speed){
	motor[GoalPusher] = speed;
}
