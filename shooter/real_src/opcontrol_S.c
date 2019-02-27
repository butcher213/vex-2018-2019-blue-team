#include "../include/main_S.h"
#include "../include/Mymotors_S.h"
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

 /* Current Driving Code 1/15/2019*/
void opcontrol() {
  double flapperPos;
  int drivingVar = 1;
  int color = 0;
  int leftStickValueX = 0;
  int leftStickValueY = 0;
  int rightStickValueX = 0;
  int rightStickValueY = 0;

  // ------------------------ red auton --------------------------------------
  if(color == 1) {
    // Load ball from the capper
    spinIntake(1);
    delay(7000);
    spinIntake(0);

    //
    loadBallsIntoCatapult();
    moveIn(18, 18);
    delay(1000);
    stopDriveMotors();
    delay(1000);
    launchCatapult();
    delay(1000);
    moveIn(-2,0);
    delay(1000);
    // push the lower flag
    moveIn(25, 25);
    delay(1000);
    moveIn(TILE_LENGTH * -2.65,TILE_LENGTH * -2.65);
    delay(500);
      moveIn(15*PI/4.0, -15*PI/4.0);
      delay(500);
      moveIn(-3,-3);
      delay(500);
      motor_move(MOTOR_FRONT_LEFT, 127);
      motor_move(MOTOR_BACK_LEFT, 127);
      motor_move(MOTOR_FRONT_RIGHT, 127);
      motor_move(MOTOR_BACK_RIGHT, 127);
      delay(1300);
      stopDriveMotors();

  //  turnDeg(90);
} else if(color == 0) {
  //spinIntake(1);
//  delay(5000);
//  spinIntake(0);
  loadBallsIntoCatapult();
  printf("hi");
  moveIn(12, 12);
//  moveIn(0-,-8);
delay(500);
  moveIn(-5,2.5);
  delay(1000);
    stopDriveMotors();
  launchCatapult();
  delay(1000);
  moveIn(-1,1);
  delay(1000);
  // push the lower flag
  moveIn(32, 32);
 delay(1000);
 moveIn(-15,-15);
 delay(500);
 moveIn(5,-4);
 delay(500);
 moveIn(-47,-47);
 moveIn(-16*PI/4.0, 16*PI/4.0);
 delay(500);
  moveIn(TILE_LENGTH * -2.7,TILE_LENGTH * -2.7);
  delay(500);
    moveIn(15*PI/4.0, -15*PI/4.0);
    delay(500);
    moveIn(-3,-3);
    delay(500);
    motor_move(MOTOR_FRONT_LEFT, 127);
    motor_move(MOTOR_BACK_LEFT, 127);
    motor_move(MOTOR_FRONT_RIGHT, 127);
    motor_move(MOTOR_BACK_RIGHT, 127);
    delay(1300);
    stopDriveMotors();

  }
  while(1){

/*  R2 - Shoot
    B - Manual Shoot Cancel
    L2(Hold) - Slow Drive
    L1(Hold) - Intake
    RIGHT - Dump Flapper

    Tank Drive:
    Left stick control left wheels, right stick controls right wheels.

    Arcade Drive:
    Left stick drives forward, backward, left, and right. Does Everything.

    Modified Arcade Drive:
    If you've played Halo 3 think of it like that. Left stick forward drives forward and backwards, right stick turns left and right.
    */


    leftStickValueX = controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X);
    leftStickValueY = controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y);
    rightStickValueX = controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X);
    rightStickValueY = controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y);
    if((abs(leftStickValueX) > 5) | (abs(leftStickValueY) > 5) | (abs(rightStickValueX) > 5) | (abs(rightStickValueY) > 5))
    {

      /* Tank Drive */

      /* R2 Holding = Slow Drive */
      /*if((controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1) && drivingVar){
        motor_move(MOTOR_BACK_LEFT, leftStickValueY / 4);
        motor_move(MOTOR_FRONT_LEFT, leftStickValueY / 4);
        motor_move(MOTOR_BACK_RIGHT, rightStickValueY / 4);
        motor_move(MOTOR_FRONT_RIGHT, rightStickValueY / 4);
      }*/

      /* Quick Drive, Slow Hold Disabled */

      /*else if(drivingVar){
        motor_move(MOTOR_BACK_LEFT, leftStickValueY);
        motor_move(MOTOR_FRONT_LEFT, leftStickValueY);
        motor_move(MOTOR_BACK_RIGHT, rightStickValueY);
        motor_move(MOTOR_FRONT_RIGHT, rightStickValueY);
      }*/

      /* Modified Arcade Drive A.K.A. Halo Drive */

      /* R2 Holding = Slow Drive */
      if((controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1) && drivingVar){
        motor_move(MOTOR_BACK_LEFT, leftStickValueY / 4);
        motor_move(MOTOR_FRONT_LEFT, leftStickValueY / 4);
        motor_move(MOTOR_BACK_RIGHT, leftStickValueY / 4);
        motor_move(MOTOR_FRONT_RIGHT, leftStickValueY / 4);

        motor_move(MOTOR_BACK_LEFT, rightStickValueX / 4);
        motor_move(MOTOR_FRONT_LEFT, rightStickValueX / 4);
        motor_move(MOTOR_BACK_RIGHT, rightStickValueX / -4);
        motor_move(MOTOR_FRONT_RIGHT, rightStickValueX / -4);
      }

      /* Quick Drive, Slow Hold Disabled */

      else if(drivingVar){

        motor_move(MOTOR_BACK_LEFT, leftStickValueY);
        motor_move(MOTOR_FRONT_LEFT, leftStickValueY);
        motor_move(MOTOR_BACK_RIGHT, leftStickValueY);
        motor_move(MOTOR_FRONT_RIGHT, leftStickValueY);

        motor_move(MOTOR_BACK_LEFT, rightStickValueX);
        motor_move(MOTOR_FRONT_LEFT, rightStickValueX);
        motor_move(MOTOR_BACK_RIGHT, rightStickValueX * -1);
        motor_move(MOTOR_FRONT_RIGHT, rightStickValueX * -1);
      }

      /* Classic Arcade Drive */

      /* R2 Holding = Slow Drive */
      /*if((controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1) && drivingVar){
        motor_move(MOTOR_BACK_LEFT, leftStickValueY / 4);
        motor_move(MOTOR_FRONT_LEFT, leftStickValueY / 4);
        motor_move(MOTOR_BACK_RIGHT, leftStickValueY / 4);
        motor_move(MOTOR_FRONT_RIGHT, leftStickValueY / 4);

        motor_move(MOTOR_BACK_LEFT, leftStickValueX / 4);
        motor_move(MOTOR_FRONT_LEFT, leftStickValueX / 4);
        motor_move(MOTOR_BACK_RIGHT, leftStickValueX / -4);
        motor_move(MOTOR_FRONT_RIGHT, leftStickValueX / -4);
      }*/

      /* Quick Drive, Slow Hold Disabled */
    /*  else if(drivingVar){
        motor_move(MOTOR_BACK_LEFT, leftStickValueY);
        motor_move(MOTOR_FRONT_LEFT, leftStickValueY);
        motor_move(MOTOR_BACK_RIGHT, leftStickValueY);
        motor_move(MOTOR_FRONT_RIGHT, leftStickValueY);

        motor_move(MOTOR_BACK_LEFT, leftStickValueX);
        motor_move(MOTOR_FRONT_LEFT, leftStickValueX);
        motor_move(MOTOR_BACK_RIGHT, leftStickValueX * -1);
        motor_move(MOTOR_FRONT_RIGHT, leftStickValueX * -1);
      }*/
    }
    else{
      motor_move(MOTOR_BACK_LEFT, 0);
      motor_move(MOTOR_FRONT_LEFT, 0);
      motor_move(MOTOR_BACK_RIGHT, 0);
      motor_move(MOTOR_FRONT_RIGHT, 0);
    }

    /* Digital Buttons */

    /* Shoot */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R2) == 1){
      printf("R2\n");
      motor_move(MOTOR_CATAPULT_LEFT, 127);
      motor_move(MOTOR_CATAPULT_RIGHT, 127);
      motor_move(MOTOR_BACK_LEFT, 0);
      motor_move(MOTOR_FRONT_LEFT, 0);
      motor_move(MOTOR_BACK_RIGHT, 0);
      motor_move(MOTOR_FRONT_RIGHT, 0);
      drivingVar = 0;
      delay(200);
    }

    /* Sensor Triggered Cancel Shooting */
    if(adi_digital_read('H') == 1){
      delay(700);
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);
      drivingVar = 1;
    }

    /* Cancell Shooting */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_B) == 1){
      printf("B\n");
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);
      drivingVar = 1;
    }

    /* Intake Hold */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L1) == 1){
      printf("L1");
      motor_move(MOTOR_INTAKE, 127);
      motor_move(MOTOR_FRONT_INTAKE, 127);
      motor_move(MOTOR_BELT, 127);
    }
    else{
      motor_move(MOTOR_INTAKE, 0);
      motor_move(MOTOR_FRONT_INTAKE, 0);
      motor_move(MOTOR_BELT, 0);
    }

    /* Flapper - Dump */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_RIGHT) == 1)
    {
      printf("Right\n");
      while(adi_digital_read('H') == 0) {
        motor_move(MOTOR_CATAPULT_LEFT, 127);
        motor_move(MOTOR_CATAPULT_RIGHT, 127);
      }
      motor_move(MOTOR_CATAPULT_LEFT, 3);
      motor_move(MOTOR_CATAPULT_RIGHT, 3);
      delay(300);
      // dump balls
      motor_move(MOTOR_FLAPPER,50);
      delay(750);
      motor_move(MOTOR_FLAPPER,-50);
      delay(750);
      motor_move(MOTOR_FLAPPER,0);
      motor_move(MOTOR_CATAPULT_LEFT, -25);
      motor_move(MOTOR_CATAPULT_RIGHT, -25);
      delay(300);
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);

    }

    //double leftPower = motor_get_power(MOTOR_CATAPULT_LEFT);
    //double rightPower = motor_get_power(MOTOR_CATAPULT_RIGHT);
    //printf("LEFT: %lf   RIGHT: %lf\n", leftPower, rightPower);




  }
}
