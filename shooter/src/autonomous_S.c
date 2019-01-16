#include "../include/main_S.h"
#include "../include/Sensors_S.h"
#include "../../include/PID.H"
#include "../include/Mymotors_S.h"
#define MAT_Size 22.1

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  // 1 for red, 0 for blue, anything else for no auton
  int color = 1;
  // ------------------------ red auton --------------------------------------
  if(color == 1) {
    // Launches preload ball and fed ball into the top targets
    spinIntake(1);
    delay(5000);
    spinIntake(0);
    loadBallsIntoCatapult();
    moveIn(TILE_LENGTH, TILE_LENGTH);
    delay(500);
    launchCatapult();
    // push the lower flag
    moveIn(TILE_LENGTH *.9, TILE_LENGTH*.9);
  } else if(color == 0) {
    // ------------------------ blue auton -------------------------------------

  } else {


  }
}
