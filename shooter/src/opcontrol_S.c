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
void opcontrol() {
  int drivingVar = 1;
  while(1){

    /* Tank Drive */
    /*if((controller_get_digital(CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R2) == 1) && drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y) / 4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y) / 4);
    }
    else if(drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y));
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y));
    }*/

    /* Modified Arcade Drive */
    /*if((controller_get_digital(CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R2) == 1) && drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / -4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) / -4);
    }
    else if(drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) * -1);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X) * -1);
    }*/

    /* Classic Arcade Drive */
    if((controller_get_digital(CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R2) == 1) && drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y) / 4);

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) / 4);
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) / 4);
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) / -4);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) / -4);
    }
    else if(drivingVar){
      motor_move(MOTOR_BACK_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y));

      motor_move(MOTOR_BACK_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X));
      motor_move(MOTOR_FRONT_LEFT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X));
      motor_move(MOTOR_BACK_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) * -1);
      motor_move(MOTOR_FRONT_RIGHT, controller_get_analog(CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X) * -1);
    }


    /* Digital Buttons */
    if(controller_get_digital(CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1){
      motor_move(MOTOR_CATAPULT_LEFT, 127);
      motor_move(MOTOR_CATAPULT_RIGHT, 127);
      motor_move(MOTOR_BACK_LEFT, 0);
      motor_move(MOTOR_FRONT_LEFT, 0);
      motor_move(MOTOR_BACK_RIGHT, 0);
      motor_move(MOTOR_FRONT_RIGHT, 0);
      drivingVar = 0;
    }
    if(adi_digital_read('H') == 1){
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);
      drivingVar = 1;
    }
    if(controller_get_digital(CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L1) == 1){
      motor_move(MOTOR_INTAKE, 127);
      motor_move(MOTOR_BELT, 127);
    }
    else{
      motor_move(MOTOR_INTAKE, 0);
      motor_move(MOTOR_BELT, 0);
    }
    if(controller_get_digital(CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_B) == 1){
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);
      drivingVar = 1;
    }
  }
}
