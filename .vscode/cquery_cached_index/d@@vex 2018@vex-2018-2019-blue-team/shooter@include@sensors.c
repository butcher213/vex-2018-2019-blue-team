#include "main.h"
//#include "Motors2.h"
/* Function:		moveIn
 * Purpose:			moves the robot a specified amount of inches
 * Argument:		inches = amount of inches to move
 * Returns:			N/A
 */
 #define PI 3.1415
void moveIn(int inches) {
  motor_set_zero_position(2,0);
  int speed = 30;
  double rightpos = 0;
  while(rightpos < (inches * (360 / (PI * 2)))) {
    /*if(rightpos > (.9 * 12 * (360/(PI * 4)))) {
      speed = - 500 * inches * (rightpos - inches);
    }*/
    rightpos =  motor_get_position(2);
//    printf("value: %f\n", motor_get_actual_velocity(2));
    double leftpos = motor_get_position(1);
    if(leftpos > rightpos) {
      motor_move(2, speed);
      motor_move(1,speed * .9);
    } else {
      motor_move(2, speed);
      motor_move(1,speed * .9);
    }
  }
  motor_set_zero_position(2,0);
  while( 0 - motor_get_actual_velocity(2) > 0) {
    motor_move(2,-50);
    motor_move(1,-50);
  }
  motor_move(1, 0);
  motor_move(2, 0);
}
/* Function:		rotateTo
 * Purpose:			rotates the robot to the specified degree
 * Argument:		deg = degree to move to
 * Return:			N/A
 */
void rotateTo(float targetDeg) {

}
void PID_control(float target) {
  motor_set_zero_position(2,0);
  motor_set_zero_position(1,0);
  float K_p = .5;
  float K_i = 0.000009;
  float K_d = 0.009;
  double integral_left = 0;
  double integral_right = 0;
  float prev_error_left = 0;
  float prev_error_right = 0;
  float pid_target = motor_get_position(1) + (target * (360 / (PI * 2)));
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
    motor_move(2,pid_speed_right);
    if(error_left == 0 && error_right == 0) {
      break;
    }
  }
}
