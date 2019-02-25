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
<<<<<<< HEAD

  //initialize();

=======
  int prevCurrentDraw =  motor_get_current_draw(MOTOR_CATAPULT_LEFT);
  int32_t diff;
  int color = 0;

  // ------------------------ red auton --------------------------------------
  if(color == 1) {
    // Load ball from the capper
    spinIntake(1);
    delay(5000);
    spinIntake(0);

    //
    loadBallsIntoCatapult();
    moveIn(18, 18);
    delay(1000);
    stopDriveMotors();
    delay(1000);
    launchCatapult();
    delay(1000);
    moveIn(-1,0);
    delay(1000);
    // push the lower flag
    moveIn(25, 25);
    delay(1000);
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
  moveIn(-3,2);
  delay(1000);
    stopDriveMotors();
  launchCatapult();
  delay(1000);
  moveIn(-2,1);
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
/*  moveIn(TILE_LENGTH * -2.7,TILE_LENGTH * -2.7);
  delay(500);
    moveIn(15*PI/4.0, -15*PI/4.0);
    delay(500);
    moveIn(-3,-3);
    delay(500);*/
    motor_move(MOTOR_FRONT_LEFT, 127);
    motor_move(MOTOR_BACK_LEFT, 127);
    motor_move(MOTOR_FRONT_RIGHT, 127);
    motor_move(MOTOR_BACK_RIGHT, 127);
    delay(1300);
    stopDriveMotors();

}
>>>>>>> e0dff5e36c92cc832dbcafea9d842925ee2e6ee8
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






    /* Tank Drive */

    /* R2 Holding = Slow Drive */
    if((controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1) && drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y) / 4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y) / 4);
    }

    /* Quick Drive, Slow Hold Disabled */

    else if(drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y));
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y));
    }

    /* Modified Arcade Drive */

    /* R2 Holding = Slow Drive */
/*    if((controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1) && drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / -4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / -4);
      printf("slow %d\n", controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
    }*/

    /* Quick Drive, Slow Hold Disabled */
/*
    else if(drivingVar){

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) * -1);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) * -1);
    }*/
//printf("fast %d\n", controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X));
    /* Classic Arcade Drive */

    /* R2 Holding = Slow Drive */
    /*if((controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1) && drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) / -4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) / -4);
    }*/

    /* Quick Drive, Slow Hold Disabled */
  /*  else if(drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) * -1);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) * -1);
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
  //  diff = 0;
    if(drivingVar == 0) {
      delay(15);
      diff =  prevCurrentDraw - motor_get_current_draw(MOTOR_CATAPULT_LEFT);
    //  printf("hi %d %d - %d\n", motor_get_current_draw(MOTOR_CATAPULT_LEFT), prevCurrentDraw, diff);
    }
    /* Sensor Triggered Cancel Shooting */
<<<<<<< HEAD
    if(adi_digital_read('H') == 1){
      delay(700);
=======
    prevCurrentDraw = motor_get_current_draw(MOTOR_CATAPULT_LEFT);

    if(diff > 150){
      delay(50);
>>>>>>> e0dff5e36c92cc832dbcafea9d842925ee2e6ee8
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
      motor_move_velocity(MOTOR_CATAPULT_LEFT, 3);
      motor_move_velocity(MOTOR_CATAPULT_RIGHT, 3);
      delay(100);
      // dump balls
<<<<<<< HEAD
      motor_move(MOTOR_FLAPPER,75);
      delay(750);
      motor_move(MOTOR_FLAPPER,-75);
=======
      motor_move(MOTOR_FLAPPER, -50);
      delay(750);
      motor_move(MOTOR_FLAPPER, 50);
>>>>>>> e0dff5e36c92cc832dbcafea9d842925ee2e6ee8
      delay(750);
      motor_move(MOTOR_FLAPPER,0);
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);

    }

    //double leftPower = motor_get_power(MOTOR_CATAPULT_LEFT);
    //double rightPower = motor_get_power(MOTOR_CATAPULT_RIGHT);
    //printf("LEFT: %lf   RIGHT: %lf\n", leftPower, rightPower);




  }
}
