
int getControllerProperty(int controllerNumber, controller_digital_e_t property) {
    return controller_get_digital(controllerNumber, property);
}

int getControllerProperty(int controllerNumber, controller_analog_e_t property) {
    return controller_get_analog(controllerNumber, property);
}
