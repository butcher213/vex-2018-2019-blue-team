#ifndef _MISC_TEMPLATE_H_
#define _MISC_TEMPLATE_H_
/*
*======================================
*						--IMPORTANT--
*
* USE #include TO IMPORT THIS TEMPLATE
*
*						--IMPORTANT--
*======================================
*/





//==========================================================================SHOW BATTERY===========================================
void showBatteryLevels(){
	bLCDBacklight = true;                                    // Turn on LCD Backlight
	string mainBattery, backupBattery;

	clearLCDLine(0);                                            // Clear line 1 (0) of the LCD
	clearLCDLine(1);                                            // Clear line 2 (1) of the LCD

	//Display the Primary Robot battery voltage
	displayLCDString(0, 0, "Primary: ");
	sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V'); //Build the value to be displayed
	displayNextLCDString(mainBattery);

	//Display the Backup battery voltage
	displayLCDString(1, 0, "Backup: ");
	sprintf(backupBattery, "%1.2f%c", BackupBatteryLevel/1000.0, 'V');    //Build the value to be displayed
	displayNextLCDString(backupBattery);
}

//==========================================================================Algorithms=============================================
//return [int] Unit Magnitude
int sign(float num){
	if(num == 0)
		return 0;
	return num/abs(num);
}
//return [float] clampped value between <float min> and <float max>
float clamp(float number, float min, float max){
	if(number > max){
		return max;
	}
	else if(number < min){
		return min;
	}
	return number;
}
//return [float] minimum value
float min(float num1, float num2){
	if(num1 < num2)
		return num1;
	return num2;
}
//return [float] maximum value
float max(float num1, float num2){
	if(num1 > num2)
		return num1;
	return num2;
}
//return [float] number no less than min
float dontLess(float num, float min){
	if(num < min)
		return min;
	return num;
}
//return [float] number no more than max
float dontMore(float num, float max){
	if(num > max)
		return max;
	return num;
}
// return [float] number raised to a non-negative integer power
long int ipow(float base, int power) {
	long int val = 1;

	for (int i = 0; i < power; i++)
		val *= base;

	return val;
}
#endif // _MISC_TEMPLATE_H_
