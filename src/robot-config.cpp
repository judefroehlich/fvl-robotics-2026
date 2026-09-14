#include "robot-config.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Update these ports to match your wiring. A negative port reverses the motor.
pros::MotorGroup left_drive({1, 2}, pros::v5::MotorGears::blue);
pros::MotorGroup right_drive({-3, -4}, pros::v5::MotorGears::blue);

pros::Imu imu(10);

void robot_config_initialize() {
    left_drive.set_brake_mode(pros::v5::MotorBrake::coast);
    right_drive.set_brake_mode(pros::v5::MotorBrake::coast);
    imu.reset(true);
}
