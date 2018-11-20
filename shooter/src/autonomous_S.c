#include "../include/main_S.h"
#include "../include/Sensors_S.h"
#include "../../include/PID.H"
#include "../include/Mymotors_S.h"
#define MAT_Size 22.1


/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  int motorPorts[] = {MOTOR_BACK_RIGHT, MOTOR_FRONT_RIGHT};
  PID_properties_t rightMotors = createPID(0.5, 0.000009, 0.009, motorPorts, 2, 40);
  PID_Properties_t leftMotors = createPID(0.5, 0.000009, 0.009, motorPorts, 2, 40);
  moveIn(3 + MAT_Size);


  //moveIn(6);
  //get_ball_from_capper();
  //load_ball();
  //shoot_balls();
  /*Assuming we shoot from the wall*/
}
