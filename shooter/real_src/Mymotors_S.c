#include "main_S.h"
#include "Mymotors_S.h"


/* Function:		initMotors
 * Purpose:			Get Motors Initialized
 * Arguments:	  motor: motor ports, gearset: gearset, reversed, should the motor be reversed
 * Returns:			N/A
 */
void initMotors(int motor, int gearset, bool reversed) {
   motor_set_gearing(motor, gearset);
   motor_set_reversed(motor, reversed);
   motor_set_encoder_units(motor, E_MOTOR_ENCODER_DEGREES);
<<<<<<< HEAD
}/* encoders not working | they do not count */
=======
  // motor_tare_position(motor);
 }
>>>>>>> d6f0cc1ad94cc6834d1d56d7020cd33875dd6a7d


/* Function:		initDrive
 * Purpose:			Get PID's Made
 * Arguments:	  Kp: P, Ki: I, Kd: D
 * Returns:			{PID_properties_t left, PID_properties_t right}
 */
// PID_array_t initDrive(double Kp, double Ki, double Kd) {
//   printf("initializing\n");
//   initMotors(MOTOR_FRONT_LEFT, E_MOTOR_GEARSET_18, 0);
//   initMotors(MOTOR_FRONT_RIGHT, E_MOTOR_GEARSET_18, 0);
//   initMotors(MOTOR_BACK_LEFT, E_MOTOR_GEARSET_18, 1);
//   initMotors(MOTOR_BACK_RIGHT, E_MOTOR_GEARSET_18, 1);
//   int left[2] = {MOTOR_FRONT_LEFT, MOTOR_BACK_LEFT};
//   int right[2] = {MOTOR_FRONT_RIGHT, MOTOR_BACK_RIGHT};
//   PID_properties_t a[2] = {createPID(Kp, Ki, Kd, left, 2, 40), createPID(Kp, Ki, Kd, right, 2, 40)};
//   return a;
// }



/* Function:		initDrive
 * Purpose:			Get PID's Made
 * Arguments:	  PIDs: PID_properties_t for left and right, left: left target, right: right target
 * Returns:			{PID_properties_t left, PID_properties_t right}
 */

PID_properties_t rightMotors, leftMotors;

void initPID() {
  static int rightMotorPorts[] = {MOTOR_BACK_RIGHT, MOTOR_FRONT_RIGHT};
//  rightMotors = createPID(0.5, 0.00009, 0.009, rightMotorPorts, 2, 40);
  rightMotors = createPID(0.5, 0.0001, 0.09, rightMotorPorts, 2, 40);
  static int leftMotorPorts[] = {MOTOR_BACK_LEFT, MOTOR_FRONT_LEFT};
//  leftMotors = createPID(0.5, 0.00009, 0.009, leftMotorPorts, 2, 40);
leftMotors = createPID(0.5, 0.0001, 0.09, leftMotorPorts, 2, 40);
}


/*void moveIn(double left, double right) {
//  left *=0.5;
//  right *=0.5;
  PID_properties_t a[2] = {generateMovedPID(leftMotors, 360/(4*PI)*left), generateMovedPID(rightMotors, 360/(4*PI)*right)};
  printf("start: %d  %d\n", a[0].error, a[1].error);
  printf("%d\n", atTarget(a[0]));
  bool flag = 0;
  //a[0].error = a[1].error;
  leftMotors = a[0];
  rightMotors = a[1];
    if(!atTarget(a[0])){
      a[0] = generateNextPID(a[0]);
    }
    if(!atTarget(a[1])){
      a[1] = generateNextPID(a[1]);
    }
    if(atTarget(a[0]) & atTarget(a[1])){
      break;
    }
  leftMotors = a[0];
  rightMotors = a[1];
  motor_move(MOTOR_FRONT_LEFT, 0);
  motor_move(MOTOR_FRONT_RIGHT, 0);
  motor_move(MOTOR_BACK_LEFT, 0);
  motor_move(MOTOR_BACK_RIGHT, 0);
}


void turnDeg(double deg) {
  double dist = deg * 180/PI * 16/2;

  moveIn(dist, -dist);

}
/* Function:		stopDriveMotors
 * Purpose:			stops all drive motors.
 * Arguments:	  N/A
 * Returns:			N/A
 */

void stopDriveMotors(void) {
  motor_move(MOTOR_BACK_LEFT, 0);
  motor_move(MOTOR_FRONT_LEFT, 0);
  motor_move(MOTOR_BACK_RIGHT, 0);
  motor_move(MOTOR_FRONT_RIGHT, 0);
}


/* Function:		launchCatapult
 * Purpose:		  Launch the catapult arm to throw a ball in autonomous
 * Arguments:	  N/A
 * Returns:			N/A
 */

void launchCatapult(void) {
  stopDriveMotors();
  printf("voltage : %d\n", motor_get_current_draw(MOTOR_CATAPULT_LEFT));
  int max = 0;
  /*while(motor_get_current_draw(MOTOR_CATAPULT_LEFT) < 1900) {
    if(motor_get_current_draw(MOTOR_CATAPULT_LEFT) > max ) {
    printf("max: %d\n", motor_get_current_draw(MOTOR_CATAPULT_LEFT));
    max = motor_get_current_draw(MOTOR_CATAPULT_LEFT);
  }*/
    motor_move(MOTOR_CATAPULT_LEFT, 127);
    motor_move(MOTOR_CATAPULT_RIGHT, 127);
  //}
  delay(750);
  //delay(50);
  motor_move(MOTOR_CATAPULT_LEFT, 0);
  motor_move(MOTOR_CATAPULT_RIGHT, 0);
}
/* Function:		spinLoader
 * Purpose:		  spins the loading mechanism to load a ball into the flapper.
 *              No exit condition
 * Arguments:	  multiplier - a number between -1 and 1 to spin the motors at.
  *             .75 for 75% and so on.
 * Returns:			N/A
 */

void spinIntake(double multiplier) {
motor_move(MOTOR_INTAKE, 127 * multiplier);
motor_move(MOTOR_FRONT_INTAKE, 127 * multiplier);
motor_move(MOTOR_BELT, 127 * multiplier);
<<<<<<< HEAD
motor_move(MOTOR_FEEDER, 127 * multiplier);
=======

>>>>>>> d6f0cc1ad94cc6834d1d56d7020cd33875dd6a7d
}

/* Function:		loadBallsIntoCatapult
 * Purpose:		  dumps balls from the flapper wrist into the catapult
 * Arguments:	  N/A
 * Returns:			N/A
 */

void loadBallsIntoCatapult(void) {
  while(adi_digital_read('H') == 0) {
    motor_move(MOTOR_CATAPULT_LEFT, 127);
    motor_move(MOTOR_CATAPULT_RIGHT, 127);
  }
  delay(25);
  motor_move(MOTOR_CATAPULT_LEFT, 10);
  motor_move(MOTOR_CATAPULT_RIGHT,10);
  delay(100);
  // dump balls
  motor_move(MOTOR_FLAPPER, -50);
  delay(750);
  motor_move(MOTOR_FLAPPER, 0);
  delay(500);
  motor_move(MOTOR_FLAPPER, 50);
  delay(750);
  motor_move(MOTOR_FLAPPER,0);
  motor_move(MOTOR_CATAPULT_LEFT, 0);
  motor_move(MOTOR_CATAPULT_RIGHT, 0);
}
