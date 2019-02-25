#include "../include/main_S.h"
#include "../include/Mymotors_S.h"
//#include "../../include/PID.h"

void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *``
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  initMotors(MOTOR_FRONT_LEFT, E_MOTOR_GEARSET_18, 0);
  initMotors(MOTOR_FRONT_RIGHT, E_MOTOR_GEARSET_36, 1);
  initMotors(MOTOR_BACK_RIGHT, E_MOTOR_GEARSET_36, 1);
  initMotors(MOTOR_BACK_LEFT, E_MOTOR_GEARSET_18, 0);
  initMotors(MOTOR_CATAPULT_LEFT, E_MOTOR_GEARSET_18, 0);
  initMotors(MOTOR_CATAPULT_RIGHT, E_MOTOR_GEARSET_18, 1);
  initMotors(MOTOR_BELT, E_MOTOR_GEARSET_18, 0);
  initMotors(MOTOR_INTAKE, E_MOTOR_GEARSET_18, 0);
  initMotors(MOTOR_FRONT_INTAKE, E_MOTOR_GEARSET_18, 1);
  initPID();
  int drivingVar = 1;
  int color = 1;
}
  // ------------------------ red auton --------------------------------------
/*  if(color == 1) {
    // Launches preload ball and fed ball into the top targets
    /*spinIntake(1);
    delay(5000);
    spinIntake(0);*/
    //loadBallsIntoCatapult();
    //moveIn(-12, -12);
    //stopDriveMotors();
    //delay(1000);
    //launchCatapult();
    //delay(1000);
    // push the lower flag
    //moveIn(TILE_LENGTH *.9, TILE_LENGTH*.9);
  //}
//}

  /* Move Inches Prototype */

  /*int left[2] = {MOTOR_FRONT_LEFT, MOTOR_BACK_LEFT};
  int right[2] = {MOTOR_FRONT_RIGHT, MOTOR_BACK_RIGHT};


  double kp = 5, ki = 0, kd = 0;

  PID_properties_t PIDs[2] = {createPID(kp,ki,kd, left, 2, 40), createPID(kp,ki,kd, right, 2, 40)};

  double leftDist = 12;
  double rightDist = 12;
  printf("go\n");
  PID_properties_t a[2] = {generateMovedPID(PIDs[0], 360/(4*PI)*leftDist), generateMovedPID(PIDs[1], 360/(4*PI)*rightDist)};
    while (true) {
      printf("%d, %f\n", atTarget(a[0]), a[0].error);
      a[0] = generateNextPID(a[0]);
      a[1] = generateNextPID(a[1]);
    }
    /*int motorPorts[] = {1,2,3,4};
    int numMotorPorts = 4;
    int startSlowingValue = 40;
    int target = 360;
    PID_properties_t ziegler = findKpid_Ziegler(motorPorts, numMotorPorts, startSlowingValue, target);*/


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
//void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
//void competition_initialize() {}
