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

 /* Current Driving Code 2/27/2019*/
void opcontrol() {
  double flapperPos;
  int color = 0;
  int leftStickValueX = 0;
  int leftStickValueY = 0;
  int rightStickValueX = 0;
  int rightStickValueY = 0;
  float leftMotorSpeed = 0;
  float rightMotorSpeed = 0;
  enum driveMode{TANK, HALO, ARCADE};
  enum driveMode driveToggle = HALO;


  while(1){


  /////////////////////////////////////////////////////////////////////////////
  /* Control Scheme Explanation */
  /////////////////////////////////////////////////////////////////////////////
  /*  R2 - Shoot
    R1(Hold) - Manual Shoot
    B - Manual Shoot Cancel
    L2(Hold) - Slow Drive
    L1(Hold) - Intake
    RIGHT - Dump Flapper

    Tank Drive:
    Left stick control left wheels, right stick controls right wheels.

    Modified Arcade Drive:
    If you've played Halo 3 think of it like that. Left stick forward drives forward and backwards, right stick turns left and right.

    Arcade Drive:
    Left stick drives forward, backward, left, and right. Does Everything.
  */

    /* Read in the values of the joysticks to control driving. */
    leftStickValueX = controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_X);
    leftStickValueY = controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y);
    rightStickValueX = controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_X);
    rightStickValueY = controller_get_analog(E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_RIGHT_Y);
    //printf("LEFT X: %d LEFT Y: %d  RIGHT X: %d RIGHT Y: %d\n", leftStickValueX, leftStickValueY, rightStickValueX, rightStickValueY);
    //printf("Drive Mode: %d\n", driveToggle);

    /* If any of the sticks are moved a decent distance past the center. Accounts for sticky or worn joysticks. */
    if((leftStickValueX > 5) || (leftStickValueY > 5) || (rightStickValueX > 5) || (rightStickValueY > 5)
      || (leftStickValueX < -5) || (leftStickValueY < -5) || (rightStickValueX < -5) || (rightStickValueY < -5))
    {

      switch(driveToggle)
      {


        ///////////////////////////////////////////////////////////////////////
        /* Drive Modes */
        ///////////////////////////////////////////////////////////////////////

        case TANK:
        /* Tank Drive */

          /* L2 Holding = Slow Drive */
          if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1)
          {
            motor_move(MOTOR_BACK_LEFT, leftStickValueY / 4);
            motor_move(MOTOR_FRONT_LEFT, leftStickValueY / 4);
            motor_move(MOTOR_BACK_RIGHT, rightStickValueY / 4);
            motor_move(MOTOR_FRONT_RIGHT, rightStickValueY / 4);
          }

          /* Quick Drive, Slow Hold Disabled */
          else
          {
            motor_move(MOTOR_BACK_LEFT, leftStickValueY);
            motor_move(MOTOR_FRONT_LEFT, leftStickValueY);
            motor_move(MOTOR_BACK_RIGHT, rightStickValueY);
            motor_move(MOTOR_FRONT_RIGHT, rightStickValueY);
          }

          break;

        case HALO:
          /* Modified Arcade Drive A.K.A. Halo Drive */

          /* Calculate motor speed. */
          leftMotorSpeed = leftStickValueY + (rightStickValueX * (leftStickValueY/(abs(leftStickValueY))));
          rightMotorSpeed = leftStickValueY - (rightStickValueX * (leftStickValueY/(abs(leftStickValueY))));

          /* Caps the motor speed to 127 to maintain functionality */
          if(leftMotorSpeed > 127)
          {
            leftMotorSpeed = 127;
          }
          if(leftMotorSpeed < -127)
          {
            leftMotorSpeed = -127;
          }
          if(rightMotorSpeed > 127)
          {
            rightMotorSpeed = 127;
          }
          if(rightMotorSpeed < -127)
          {
            rightMotorSpeed = -127;
          }

          /* L2 Holding = Slow Drive */

          if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1)
          {
            motor_move(MOTOR_BACK_LEFT, leftMotorSpeed / 4);
            motor_move(MOTOR_FRONT_LEFT, leftMotorSpeed / 4);
            motor_move(MOTOR_BACK_RIGHT, rightMotorSpeed / 4);
            motor_move(MOTOR_FRONT_RIGHT, rightMotorSpeed / 4);
          }

          /* Quick Drive, Slow Hold Disabled */

          else
          {
            motor_move(MOTOR_BACK_LEFT, leftMotorSpeed);
            motor_move(MOTOR_FRONT_LEFT, leftMotorSpeed);
            motor_move(MOTOR_BACK_RIGHT, rightMotorSpeed);
            motor_move(MOTOR_FRONT_RIGHT, rightMotorSpeed);
          }



          break;

        case ARCADE:
        /* Classic Arcade Drive */

          /* Calculate motor speed. */
          leftMotorSpeed = leftStickValueY + (leftStickValueX * (leftStickValueY/(abs(leftStickValueY))));
          rightMotorSpeed = leftStickValueY - (leftStickValueX * (leftStickValueY/(abs(leftStickValueY))));

          /* Caps the motor speed to 127 to maintain functionality*/
          if(leftMotorSpeed > 127)
          {
            leftMotorSpeed = 127;
          }
          if(leftMotorSpeed < -127)
          {
            leftMotorSpeed = -127;
          }
          if(rightMotorSpeed > 127)
          {
            rightMotorSpeed = 127;
          }
          if(rightMotorSpeed < -127)
          {
            rightMotorSpeed = -127;
          }

          /* L2 Holding = Slow Drive */
          if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L2) == 1)
          {
            motor_move(MOTOR_BACK_LEFT, leftMotorSpeed / 4);
            motor_move(MOTOR_FRONT_LEFT, leftMotorSpeed / 4);
            motor_move(MOTOR_BACK_RIGHT, rightMotorSpeed / 4);
            motor_move(MOTOR_FRONT_RIGHT, rightMotorSpeed / 4);
          }

          /* Quick Drive, Slow Hold Disabled */
          else
          {
            motor_move(MOTOR_BACK_LEFT, leftMotorSpeed);
            motor_move(MOTOR_FRONT_LEFT, leftMotorSpeed);
            motor_move(MOTOR_BACK_RIGHT, rightMotorSpeed);
            motor_move(MOTOR_FRONT_RIGHT, rightMotorSpeed);
          }

          break;

        }
      }

    /* If the sticks are about centered then don't drive */
    else
    {
      motor_move(MOTOR_BACK_LEFT, 0);
      motor_move(MOTOR_FRONT_LEFT, 0);
      motor_move(MOTOR_BACK_RIGHT, 0);
      motor_move(MOTOR_FRONT_RIGHT, 0);
    }


    ///////////////////////////////////////////////////////////////////////////
    /* Digital Buttons */
    ///////////////////////////////////////////////////////////////////////////

    /* Shoot */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R2) == 1)
    {
      motor_move(MOTOR_CATAPULT_LEFT, 127);
      motor_move(MOTOR_CATAPULT_RIGHT, 127);
      motor_move(MOTOR_BACK_LEFT, 0);
      motor_move(MOTOR_FRONT_LEFT, 0);
      motor_move(MOTOR_BACK_RIGHT, 0);
      motor_move(MOTOR_FRONT_RIGHT, 0);
    }

    /* Sensor Triggered Cancel Shooting */
    if((adi_digital_read('H') == 1) && (controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R1) == 0))
    {
      motor_move(MOTOR_BACK_LEFT, 0);
      motor_move(MOTOR_FRONT_LEFT, 0);
      motor_move(MOTOR_BACK_RIGHT, 0);
      motor_move(MOTOR_FRONT_RIGHT, 0);
      delay(900);
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);
    }

    /* Sensor Stuck Failsafe */
    else if((adi_digital_read('H') == 1) && (controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_R1) == 1))
    {
      motor_move(MOTOR_CATAPULT_LEFT, 75);
      motor_move(MOTOR_CATAPULT_RIGHT, 75);
    }

    /* Cancel Shooting */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_B) == 1)
    {
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);
    }

    /* Intake Hold */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_L1) == 1)
    {
      motor_move(MOTOR_INTAKE, 127);
      motor_move(MOTOR_FRONT_INTAKE, 127);
      motor_move(MOTOR_BELT, 100);
    }
    else
    {
      motor_move(MOTOR_INTAKE, 0);
      motor_move(MOTOR_FRONT_INTAKE, 0);
      motor_move(MOTOR_BELT, 0);
    }

    /* Flapper - Dump */
    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_RIGHT) == 1)
    {
      while(adi_digital_read('H') == 0)
      {
        motor_move(MOTOR_CATAPULT_LEFT, 127);
        motor_move(MOTOR_CATAPULT_RIGHT, 127);
      }
      motor_move(MOTOR_CATAPULT_LEFT, 3);
      motor_move(MOTOR_CATAPULT_RIGHT, 3);
      delay(300);
      // dump balls
      motor_move(MOTOR_FLAPPER, -10);
      delay(1000);
      motor_move(MOTOR_FLAPPER, 75);
      delay(200);
      motor_move(MOTOR_FLAPPER, 0);
      motor_move(MOTOR_CATAPULT_LEFT, -35);
      motor_move(MOTOR_CATAPULT_RIGHT, -35);
      delay(400);
      motor_move(MOTOR_CATAPULT_LEFT, 0);
      motor_move(MOTOR_CATAPULT_RIGHT, 0);
    }

    if(controller_get_digital(E_CONTROLLER_MASTER, E_CONTROLLER_DIGITAL_LEFT) == 1)
    {
      driveToggle = (++driveToggle) % 3;
    }
  }
}
