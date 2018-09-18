#include "Motors.h"
#include "Controller_Template.h"
#include "Sensors.h"

void doTeleop() {
	//    MAIN     |    PART
	// 5 6 7 8 L R | 5 6 7 8 L R
  // _ _ U U _ _ | _ _ _ U X X
  // _ _ _ _ _ _ | _ _ _ _ Y Y
  //     L _     |     _ L
  //     _ R     |     R _

  // 															| B7D B8D
	float multMain = (getB7D() || getB8D())? 0.5: 1;
	//															| P7D P8D
	float multPartner = (getP7D() || getP8D())? 0.5: 1;
  if (getB8D())
  	stopAllMotors();
  else {
		// (up, down, lock) 						| P5U P5D P7U P7L
	 	coneArmControl(getP5U(), getP5D(), getP7L(), multPartner);
	 	// (open, close)								| P6D P6U
	 	coneClawControl(getP6D(), getP6U(), multPartner);
		// (up, down)										| 6U  6D
	 	goalArmControl1(getB6U(), getB6D(), multMain);
	  // (left x-axis, left y-axis,		| LJX LJY RJX RJY
	 	// right x-axis, right y-axis)
	 	wheelControl(getLJoyX(), getRJoyY(), getRJoyX(), getLJoyY(), multMain);
	 	// (open, close)								| B5U B5D
	 	pusherControl(getB5U(), getB5D(), multMain);
 	}
}

task feederArm() {
	coneClawSpeed(CONE_CLAW_OPEN);
	moveArmToPosition(CONE_ARM_FEEDER);
	coneClawSpeed(CONE_CLAW_CLOSE);
	// has cone
	moveArmToPosition(CONE_ARM_HIGH);
	coneArmSpeed(CONE_ARM_UP);
	delay(100);
	coneArmSpeed(0);
	// cone on base
	coneClawSpeed(CONE_CLAW_OPEN);
	moveArmToPosition(CONE_ARM_HIGH);
}
void coneArmControl(const bool moveUp, const bool moveDown, const bool lockArm, const float multiplier) {

	if(moveUp) {
		coneArmSpeed(CONE_ARM_UP * multiplier);
	}
	else if(moveDown) {
		coneArmSpeed(CONE_ARM_DOWN * multiplier);
	}
	else if (lockArm) {
		if(getArmPosition() < CONE_ARM_HIGH)
			coneArmSpeed(CONE_ARM_UP * .1);
		else
			coneArmSpeed(CONE_ARM_DOWN * .1);
	}
	else if (getPRJoyY() > 10) {
		coneArmSpeed(getPRJoyY());
  }
  else {
    coneArmSpeed(0);
  }

  if (getP8R()) {
  	//moveArmToPosition(CONE_ARM_FEEDER);
  	//startTask(feederArm, 15);
  }
 	else if(getP8D()) {
 		//stopTask(feederArm);
	}

}

float applyCurve(float input, int n) {
	return ipow(input, n)/ipow(127,n-1);
}
void wheelControl(int leftXAxis, int leftYAxis, int rightXAxis, int rightYAxis, const float multiplier) {
	const int threshhold = 10;


	/* -Tank- */
	rightWheels(rightYAxis * multiplier);
	leftWheels(leftYAxis * multiplier);
	if(getB8L())
		strafeWheel(STRAFE_RIGHT * multiplier);
	else if(getB7R())
		strafeWheel(STRAFE_LEFT * multiplier);
	else
		strafeWheel(0);

	/* -Arcade- */
	//rightWheels(applyCurve(leftYAxis - rightXAxis, 2) * multiplier);
	//leftWheels(applyCurve(leftYAxis + rightXAxis, 2) * multiplier);
	//if(abs(leftXAxis) > threshhold)
	//	strafeWheel(-leftXAxis * multiplier);
	//else
	//	strafeWheel(0);
}

void goalArmControl1(const bool moveUp, const bool moveDown, const float multiplier) {
	if(moveUp)
		goalArmSpeed(GOAL_ARM_UP * multiplier);
	else if(moveDown)
		goalArmSpeed(GOAL_ARM_DOWN * multiplier);
	else
		goalArmSpeed(0);
}

void coneClawControl(const bool open, const bool close, const float multiplier){
	//static bool toggleState = true, clawClosed = false;
	//if(toggleState && getB6D()) {
	//	toggleState = false;
	//	if(clawClosed) {
	//		clawSpeed(0);
	//		clawClosed = false;
	//	}
	//	else {
	//		clawSpeed(CLAW_CLOSE * multiplier);
	//		clawClosed = true;
	//	}
	//}
	//else if(!getB6D())
	//	toggleState = true;

	if(open)
		coneClawSpeed(CONE_CLAW_OPEN * multiplier);
	else if(close)
		coneClawSpeed(CONE_CLAW_CLOSE * multiplier);
	else
		coneClawSpeed(0);
}

void pusherControl(const bool open, const bool close, const float multiplier) {
	if (open)
		goalPusherSpeed(GOAL_PUSHER_OPEN * multiplier);
	else if (close)
		goalPusherSpeed(GOAL_PUSHER_CLOSE * multiplier);
	else
		goalPusherSpeed(0);

}
