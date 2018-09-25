#include "main.h"

void initMotor(int motorPort, int direction) {
  motor_set_gearing(motorPort, E_MOTOR_GEARSET_18);
  if(direction > 0)
    motor_set_reversed(motorPort, true);
  else
    motor_set_reversed(motorPort, false);
  motor_set_encoder_units(motorPort, E_MOTOR_ENCODER_DEGREES);

}
