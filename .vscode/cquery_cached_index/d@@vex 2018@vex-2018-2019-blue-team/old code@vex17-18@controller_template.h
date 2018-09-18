#ifndef _CONTROLLER_TEMPLATE_H_
#define _CONTROLLER_TEMPLATE_H_
/*
*======================================
*						--IMPORTANT--
*
* USE #include TO IMPORT THIS TEMPLATE
*
*						--IMPORTANT--
*======================================
*/




//==========================================================================Controller Commands====================================
//return [bool] Button 5
bool getB5D(){
	return vexRT[Btn5D];
}
bool getB5U(){
	return vexRT[Btn5U];
}

bool getP5D(){
	return vexRT[Btn5DXmtr2];
}
bool getP5U(){
	return vexRT[Btn5UXmtr2];
}

//return [bool] Button 6
bool getB6D(){
	return vexRT[Btn6D];
}
bool getB6U(){
	return vexRT[Btn6U];
}

bool getP6D(){
	return vexRT[Btn6DXmtr2];
}
bool getP6U(){
	return vexRT[Btn6UXmtr2];
}

//return [bool] Button 7
bool getB7D(){
	return vexRT[Btn7D];
}
bool getB7U(){
	return vexRT[Btn7U];
}
bool getB7L(){
	return vexRT[Btn7L];
}
bool getB7R(){
	return vexRT[Btn7R];
}

bool getP7D(){
	return vexRT[Btn7DXmtr2];
}
bool getP7U(){
	return vexRT[Btn7UXmtr2];
}
bool getP7L(){
	return vexRT[Btn7LXmtr2];
}
bool getP7R(){
	return vexRT[Btn7RXmtr2];
}

//return [bool] Button 8
bool getB8D(){
	return vexRT[Btn8D];
}
bool getB8U(){
	return vexRT[Btn8U];
}
bool getB8L(){
	return vexRT[Btn8L];
}
bool getB8R(){
	return vexRT[Btn8R];
}

bool getP8D(){
	return vexRT[Btn8DXmtr2];
}
bool getP8U(){
	return vexRT[Btn8UXmtr2];
}
bool getP8L(){
	return vexRT[Btn8LXmtr2];
}
bool getP8R(){
	return vexRT[Btn8RXmtr2];
}

//return [int] Joystick Left
int getLJoyX(){
	return vexRT[Ch4];
}
int getLJoyY(){
	return vexRT[Ch3];
}

int getPLJoyX(){
	return vexRT[Ch4Xmtr2];
}
int getPLJoyY(){
	return vexRT[Ch3Xmtr2];
}

//return [int] Joystick Right
int getRJoyX(){
	return vexRT[Ch1];
}
int getRJoyY(){
	return vexRT[Ch2];
}

int getPRJoyX(){
	return vexRT[Ch1Xmtr2];
}
int getPRJoyY(){
	return vexRT[Ch2Xmtr2];
}

#endif // _CONTROLLER_TEMPLATE_H_
