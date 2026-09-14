#include "main.h"

/**
 * Runs the driver control period. Arcade drive on the left stick (throttle)
 * and right stick (turn).
 */
void opcontrol() {
    while (true) {
        int power = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        left_drive.move(power + turn);
        right_drive.move(power - turn);

        pros::delay(20);
    }
}
