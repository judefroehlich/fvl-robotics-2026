#include "main.h"

/**
 * Runs once when the program starts, before autonomous or opcontrol.
 */
void initialize() {
    pros::lcd::initialize();
    robot_config_initialize();
    pros::lcd::set_text(1, "Override - Ready");
}

/**
 * Runs while the robot is disabled by the field/competition switch.
 */
void disabled() {}

/**
 * Runs after initialize() when connected to competition control, before
 * autonomous or driver control periods begin.
 */
void competition_initialize() {}
