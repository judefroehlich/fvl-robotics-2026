#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

#include "api.h"

extern pros::Controller controller;

// Drivetrain. Ports are negative where the motor is reversed.
extern pros::MotorGroup left_drive;
extern pros::MotorGroup right_drive;

extern pros::Imu imu;

void robot_config_initialize();

#endif  // _ROBOT_CONFIG_H_
