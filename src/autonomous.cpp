#include "main.h"

/**
 * Runs the autonomous period (up to 15/20s, no controller input).
 */
void autonomous() {
    left_drive.move(80);
    right_drive.move(80);
    pros::delay(1000);

    left_drive.move(0);
    right_drive.move(0);
}
