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
    /*if((controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1) && drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y) / 4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y) / 4);
    }*/

    /* Quick Drive, Slow Hold Disabled */

    /*else if(drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y));
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y));
    }*/

    /* Modified Arcade Drive */

    /* R2 Holding = Slow Drive */
    if((controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1) && drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / -4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / -4);
    }

    /* Quick Drive, Slow Hold Disabled */

    else if(drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) * -1);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) * -1);
    }

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
    /*else if(drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) * -1);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) * -1);
    }*/


    /* Digital Buttons */

    /* Shoot */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R2) == 1){
      motor_move(MOTOR_CATAPULT_LEFT, 127);
      motor_move(MOTOR_CATAPULT_RIGHT, 127);
      motor_move(MOTOR_BACK_LEFT, 0);
      motor_move(MOTOR_FRONT_LEFT, 0);
      motor_move(MOTOR_BACK_RIGHT, 0);
      motor_move(MOTOR_FRONT_RIGHT, 0);
      drivingVar = 0;
    }

    /* Sensor Triggered Cancel Shooting */
    if(adi_digital_read('H') == 1){
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);
      drivingVar = 1;
    }

    /* Cancell Shooting */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_B) == 1){
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);
      drivingVar = 1;
    }

    /* Intake Hold */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L1) == 1){
      motor_move(MOTOR_INTAKE, 127);
      motor_move(MOTOR_BELT, 127);
    }
    else{
      motor_move(MOTOR_INTAKE, 0);
      motor_move(MOTOR_BELT, 0);
    }

    /* Flapper - Dump */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_RIGHT) == 1)
    {
      while(adi_digital_read('G') == 0) {
        motor_move(MOTOR_CATAPULT_LEFT, 127);
        motor_move(MOTOR_CATAPULT_RIGHT, 127);
      }
      motor_move_velocity(MOTOR_CATAPULT_LEFT, 1);
      motor_move_velocity(MOTOR_CATAPULT_RIGHT, 1);
      delay(100);
      // dump balls
      motor_move_relative(MOTOR_FLAPPER, 220, 60);
      delay(500);
      motor_move_relative(MOTOR_FLAPPER, -220, 75);
      delay(500);
      motor_move(MOTOR_FLAPPER,0);
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);

    }


  }
}
