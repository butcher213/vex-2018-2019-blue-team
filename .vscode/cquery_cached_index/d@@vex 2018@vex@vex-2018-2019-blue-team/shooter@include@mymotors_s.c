#include "main_S.h"
#include "Mymotors_S.h"


/* Function:		initMotors
 * Purpose:			Get Motors Initialized
 * Arguments:	  motor: motor ports, gearset: gearset, reversed, should the motor be reversed
 * Returns:			N/A
 */
// void initMotors(int motor, int gearset, bool reversed) {
//   motor_set_gearing(motor, gearset);
//   motor_set_reversed(motor, reversed);
//   motor_set_encoder_units(motor, E_MOTOR_ENCODER_DEGREES);
// }


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
// PID_array_t moveIn(PID_properties_t *PIDs, double left, double right) {
//   PID_properties_t a[2] = {generateMovedPID(PIDs[0], 360/(4*PI)*left), generateMovedPID(PIDs[1], 360/(4*PI)*right)};
//   printf("%d\n", atTarget(a[0]));
//   while (!atTarget(a[0]) && !atTarget(a[1])) {
//     a[0] = generateMovedPID(PIDs[0], 360/(4*PI)*left);
//     a[1] = generateMovedPID(PIDs[1], 360/(4*PI)*right);
//   }
//   return a;
// }


/* Function:		leftWheels
 * Purpose:			moves the left side of the robot forward
 * Arguments:	  speed: the speed of the motors
 * Returns:			N/A
 */
 /*
void myleftWheels(float speed) {
  motor_move(MOTOR_LEFT, speed);
}
*/

/* Function:		rightWheels
 * Purpose:			moves the right side of the robot forward
 * Arguments:	  speed: the speed of the motors
 * Returns:			N/A
 */
 /*
void myrightWheels(float speed) {
  motor_move(MOTOR_RIGHT, speed);
}
*/
