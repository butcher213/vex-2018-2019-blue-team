#include "main_S.h"
#ifndef PI
#define PI 3.1415
#endif
/* Function:		moveIn
 * Purpose:			moves the robot a specified amount of inches
 * Argument:		inches = amount of inches to move
 * Returns:			N/A
 */

/* Function:		rotateTo
 * Purpose:			rotates the robot to the specified degree
 * Argument:		deg = degree to move to
 * Return:			N/A
 */
 void rotateTo(float targetDeg) {
    float Wheel_Spread = 14.625;
    float Max_Speed = 63;
    motor_move_relative(1, ((Wheel_Spread*PI*targetDeg)/(4*PI)), Max_Speed);
    motor_move_relative(2, -((Wheel_Spread*PI*targetDeg)/(4*PI)), Max_Speed);
  }

/* Function:		PID_control
 * Purpose:     ???
 * Argument     N/A
 * Return       N/A
 */

void PID_control() {
 /* float K_p = .5;
  float K_i = 0;
  float K_d = 0;
  motor_set_zero_position(2,0);
  motor_set_zero_position(1,0);
  float K_p = .5;
  float K_i = 0.000009;
  float K_d = 0.009;
  double integral_left = 0;
  double integral_right = 0;
  float prev_error_left = 0;
  float prev_error_right = 0;
  float pid_target = (40 * (360 / (PI * 2)));
  while(1) {
    double error_left = pid_target - motor_get_position(1);
    printf("error left %.2f\n", error_left);
    double error_right = pid_target - motor_get_position(2);
    printf("error right %.2f\n", error_right);
    integral_left += error_left;
    integral_right += error_right;
    if (error_left == 0 || abs(error_left) > 40){
      integral_left = 0;
    }
    if (error_right == 0 || abs(error_right) > 40){
      integral_right = 0;
    }
    double deriv_left = error_left - prev_error_left;
    prev_error_left = error_left;
    float pid_speed_left = K_p * error_left + (K_i * integral_left + K_d * deriv_left);
    printf("%f left speed\n", pid_speed_left);
    double deriv_right = error_right - prev_error_right;
    prev_error_right = error_right;
    float pid_speed_right = K_p * error_right + (K_i * integral_right + K_d * deriv_right);
    printf("%f right speed\n", pid_speed_right);
    pid_speed_right = K_p * error_right + K_i * integral_right + K_d * deriv_right;
    motor_move(1,pid_speed_left);
    motor_move(1,pid_speed_right);
    motor_move(2,pid_speed_right);
  }
  */
}
