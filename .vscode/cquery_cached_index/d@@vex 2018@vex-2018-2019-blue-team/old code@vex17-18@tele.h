#ifndef _TELE_H_
#define _TELE_H_



void doTeleop();
void coneArmControl(const bool moveUp, const bool moveDown, const bool lockArm, const float multiplier);
float applyCurve(float input, int n);
void wheelControl(int leftXAxis, int leftYAxis, int rightXAxis, int rightYAxis, const float multiplier);
void goalArmControl1(const bool moveUp, const bool moveDown, const float multiplier);
void coneClawControl(const bool open, const bool close, const float multiplier);
void goalArmControl(const bool moveUp, const bool moveDown, const float multiplier);
//void goalClawControl(); // depricated
void pusherControl(const bool open, const bool close, const float multiplier);

#include "Tele.c"

#endif // _TELE_H_
