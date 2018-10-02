#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_


/* Function:		getControllerProperty
 * Purpose:         Overlay for controller buttons and joysticks
 * Argument:        controllerNumber = Number for controller. 1 for main, 2 for
                        partner
                    property = the enum value under controller_analog_e_t or
                        controller_digital_e_t
 * Return:          value assosiated with property for the controller
                        controllerNumber
 */
int getControllerProperty(int controllerNumber, int property);


#include "Controller.c"
#endif // _CONTROLLER_H_
