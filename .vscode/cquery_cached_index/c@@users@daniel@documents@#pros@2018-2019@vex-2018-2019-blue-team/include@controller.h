#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "pros/misc.h"

 /* Function:		getControllerProperty
  * Purpose:        
  * Argument:
  * Return:
  */
 int getControllerProperty(int controllerNumber, controller_digital_e_t property);

 /* Function:		getControllerProperty
  * Purpose:
  * Argument:
  * Return:
  */
 int getControllerProperty(int controllerNumber, controller_analog_e_t property);


#include "Controller.c"
#endif // _CONTROLLER_H_
