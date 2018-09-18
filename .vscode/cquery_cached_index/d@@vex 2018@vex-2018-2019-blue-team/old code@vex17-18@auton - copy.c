#include "Sensors.h"
#include "Misc_Template.h"
#include "Auton.h"
#include "Motors.h"
//include "vision.h"
#define feeder_station 0
#define blue 1
void doAuton() {
	// decide if we are blue or red
//	static int startPosition = getStartPosition();
	if(blue == 1) {
		blueAuton();
	} else {
		redAuton();
	}
}
// main auton functions
void blueAuton() {

	retrieveMobile();
	//if(feeder_station == 1) {
	//	moveToFeederStation();
	//	depositFirstStack();
//	}
	// not stacking
//	rotateTo(0);
	moveIn(40);
	rotateTo(240);
	moveIn(-32);
	rotateTo(270);
	driveStraight(DRIVE_BACKWARD);
	delay(2000);
	driveStraight(0);
	coneClawSpeed(CONE_CLAW_OPEN);
	coneArmSpeed(CONE_ARM_DOWN);
	delay(500);
	coneClawSpeed(0);
	delay(1500);
	coneArmSpeed(0);
	lowerBase();
	delay(500);
	driveStraight(DRIVE_FORWARD);
	delay(1000);
	driveStraight(0);

	//depositBaseNoFeeder();
}
void redAuton() {
	retrieveMobileRed();
	//if(feeder_station == 1) {
	//	moveToFeederStationRed();
	//	depositFirstStackRed();
	//}
	// not stacking
		moveIn(40);
	rotateTo(-210);
	moveIn(-32);
	rotateTo(-270);
	driveStraight(DRIVE_BACKWARD);
	delay(2000);
	driveStraight(0);
	coneClawSpeed(CONE_CLAW_OPEN);
	coneArmSpeed(CONE_ARM_DOWN);
	delay(500);
	coneClawSpeed(0);
	delay(1500);
	coneArmSpeed(0);
	lowerBase();
	delay(500);
	driveStraight(DRIVE_FORWARD);
	delay(1000);
	driveStraight(0);
}
void deployPreload() {
	moveArmToPosition(GROUND_POS);
}
void stackPreload() {
	moveArmToPosition(STACK_POS);
}
int getStartPosition() {
	if (SensorValue[redOrBlue])
		return RED;
	return BLUE;
}

void retrieveMobile() {
leftWheels(DRIVE_BACKWARD);
delay(600);
leftWheels(0);
strafeWheel(127);
delay(500);
strafeWheel(-127);
delay(50);
strafeWheel(0);
strafeWheel(45);
moveIn(-44);
strafeWheel(0);
strafeWheel(-127);
delay(300);
strafeWheel(0);
delay(500);
coneArmSpeed(CONE_ARM_DOWN);
delay(2000);
//deployPreload();
coneArmSpeed(0);
//	resetGyro();
lowerBase();
//leftWheels(-DRIVE_BACKWARD*.6);
delay(800);
leftWheels(0);
moveIn(-22);

raiseBase();
coneArmSpeed(CONE_ARM_UP*.8);
delay(2000);
coneArmSpeed(0);
//stackPreload();
strafeWheel(-127);
delay(500);
//strafeWheel(-127);
//delay(1000);
strafeWheel(0);
//coneClawSpeed(CONE_CLAW_OPEN);
//delay(200);
//coneArmSpeed(CONE_ARM_DOWN);/
//delay(500);
//coneClawS
}
void retrieveMobileRed() {
	rightwheels(DRIVE_BACKWARD);
delay(600);
rightWheels(0);
strafeWheel(-127);
delay(500);
strafeWheel(127);
delay(50);
strafeWheel(0);
strafeWheel(-45);
moveIn(-42);
strafeWheel(0);
strafeWheel(127);
delay(300);
strafeWheel(0);
delay(500);
coneArmSpeed(CONE_ARM_DOWN);
delay(2000);
//deployPreload();
coneArmSpeed(0);
//	resetGyro();
lowerBase();
//leftWheels(-DRIVE_BACKWARD*.6);
delay(800);
leftWheels(0);
moveIn(-22);

raiseBase();
coneArmSpeed(CONE_ARM_UP*.8);
delay(2000);
coneArmSpeed(0);
//stackPreload();
strafeWheel(127);
delay(500);
//strafeWheel(-127);
//delay(1000);
strafeWheel(0);
}

void stackConeFromGround() {
//coneArmSpeed(CONE_ARM_UP);
//	delay(750);
//	coneArmSpeed(CONE_ARM_DOWN * .1);
}

void moveToFeederStation() {
	moveIn(16);
	rotateTo(120);
	delay(500);
	moveIn(-18);
	//strafeWheel(60);
	//moveIn(24);
	//strafeWheel(0);
	//delay(500);
	//rotateTo(135);
	//delay(500);
	//moveIn(-10);
}
void depositBaseNoFeeder() {
//	rotateTo(150);
	moveIn(40);
}
void moveToFeederStationRed() {
	moveIn(16);
	rotateTo(-120);
	delay(500);
	moveIn(-18);
	//strafeWheel(60);
	//moveIn(24);
	//strafeWheel(0);
	//delay(500);
	//rotateTo(135);
	//delay(500);
	//moveIn(-10);
	coneArmSpeed(0);
}
void stackConeFromFeeder() {
	for(int i = 0; i < 3; i++) {
	moveArmToPosition(STACK_POS + (i*25));
	delay(200);
	coneClawSpeed(CONE_CLAW_OPEN);
	delay(200);
	moveArmToPosition(STACK_REL_POS + (i*25));
	coneClawSpeed(0);
	delay(500);
	coneClawSpeed(0);
	setArmToFeeder();
}
}
void setArmToFeeder() {
	moveArmToPosition(FEEDER_POS);
	coneClawSpeed(CONE_CLAW_OPEN);
	delay(200);
	coneArmSpeed(CONE_ARM_UP * .05);
//	moveArmToPosition(FEEDER_POS);
	delay(300);
	coneClawSpeed(-CONE_CLAW_OPEN);
	delay(300);
	coneClawSpeed(0);
	coneArmSpeed(CONE_ARM_UP * .1);
}

void depositFirstStack() {
	rotateTo(180);

	moveIn(-40);
	delay(1000);
	driveStraight(DRIVE_BACKWARD);
	delay(1000);
	driveStraight(0);
	//lowerBase();

//	moveIn(12);
}
void depositFirstStackRed() {
	rotateTo(180);

	moveIn(-40);
	delay(1000);
	driveStraight(DRIVE_BACKWARD);
	delay(1000);
	driveStraight(0);
	//lowerBase();

//	moveIn(12);
}
void turnForDegrees(double angle) {
	angle = angle * 10;
	//while(1) {
	//	writeDebugStreamLine("%i", getGyro());
	//}
	while(getGyro() < angle) {
		leftWheels(50);
		rightWheels(-50);
	}
	while(getGyro() > angle) {
		leftWheels(-50);
		rightWheels(50);
	}
}

void lowerBase() {
	goalArmSpeed(GOAL_ARM_DOWN);
	delay(550);
	goalArmSpeed(0);
}

void raiseBase() {
	while(!getGoalLiftBump()) {
		writeDebugStreamLine("%d", getGoalLiftBump());
		goalArmSpeed(GOAL_ARM_UP);
	}
	goalArmSpeed(0);
}
void placeBase() {
	goalArmSpeed(GOAL_ARM_DOWN);
	delay(400);
	goalArmSpeed(0);
	delay(400);
	goalPusherSpeed(GOAL_PUSHER_OPEN);
	delay(1000);
	goalPusherSpeed(GOAL_PUSHER_CLOSE);
	delay(1000);
	goalPusherSpeed(0);
}
