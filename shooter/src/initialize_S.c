#include "../include/main_S.h"
#include "../include/Init_S.h"
#include "../include/Mymotors_S.h"
#include "../../include/PID.h"

void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *``
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  motor_set_gearing(MOTOR_FRONT_LEFT, E_MOTOR_GEARSET_18);
  motor_set_reversed(MOTOR_FRONT_LEFT, 0);
  motor_set_encoder_units(MOTOR_FRONT_LEFT, E_MOTOR_ENCODER_DEGREES);
  motor_set_gearing(MOTOR_FRONT_RIGHT, E_MOTOR_GEARSET_18);
  motor_set_reversed(MOTOR_FRONT_RIGHT, 1);
  motor_set_encoder_units(MOTOR_FRONT_RIGHT, E_MOTOR_ENCODER_DEGREES);
  motor_set_gearing(MOTOR_BACK_RIGHT, E_MOTOR_GEARSET_18);
  motor_set_reversed(MOTOR_BACK_RIGHT, 1);
  motor_set_encoder_units(MOTOR_BACK_RIGHT, E_MOTOR_ENCODER_DEGREES);
  motor_set_gearing(MOTOR_BACK_LEFT, E_MOTOR_GEARSET_18);
  motor_set_reversed(MOTOR_BACK_LEFT, 0);
  motor_set_encoder_units(MOTOR_BACK_LEFT, E_MOTOR_ENCODER_DEGREES);


  int left[2] = {MOTOR_FRONT_LEFT, MOTOR_BACK_LEFT};
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
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
