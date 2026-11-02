#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include <cstdio>

// extern "C" void display_initialize(void);
// auto keep_display = &display_initialize;

using namespace pros;

// Prints to the brain screen like printf, but with a line number first.
// Example: bprint(0, "X: %8.2f", x);
template <typename... Args>
void bprint(int line, const char *fmt, Args... args) {
  screen::print(E_TEXT_MEDIUM, line, fmt, args...);
}

// main gears 36 teeth, large is 48, small is 12.

// left motor group
MotorGroup left_motor_group({8, 9, 10}, MotorGears::blue);
// right motor group
MotorGroup right_motor_group({-18, -19, -20}, MotorGears::blue);

// blue motors only, used for odometry (the 5.5W motors spin at a different
// speed)
MotorGroup left_odom_motors({8, 10}, MotorGears::blue);
MotorGroup right_odom_motors({-18, -20}, MotorGears::blue);

// controller
Controller controller(E_CONTROLLER_MASTER);

// drivetrain settings
lemlib::Drivetrain
    drivetrain(&left_motor_group,          // left motor group
               &right_motor_group,         // right motor group
               10,                         // 10 inch track width
               lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
               450,                        // drivetrain rpm is 450
               2                           // horizontal drift is 2 (for now)
    );

// imu
Imu imu(13);

// tracking wheels built from the blue drive motors
lemlib::TrackingWheel left_tracking_wheel(&left_odom_motors,
                                          lemlib::Omniwheel::NEW_325, -5, 450);
lemlib::TrackingWheel right_tracking_wheel(&right_odom_motors,
                                           lemlib::Omniwheel::NEW_325, 5, 450);

// odometry settings
lemlib::OdomSensors sensors(&left_tracking_wheel,  // vertical tracking wheel 1
                            &right_tracking_wheel, // vertical tracking wheel 2
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to
                                     // nullptr as we don't have a second one
                            nullptr  //&imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(10,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       3,   // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(2,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       10,  // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       0    // maximum acceleration (slew)
    );

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve
    throttle_curve(3,    // joystick deadband out of 127
                   10,   // minimum output where drivetrain will move out of 127
                   1.019 // expo curve gain
    );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steer_curve(3,    // joystick deadband out of 127
                10,   // minimum output where drivetrain will move out of 127
                1.019 // expo curve gain
    );

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);

// initialize function. Runs on program startup
void initialize() {
  printf("On\n");
  chassis.calibrate(); // calibrate sensors
  // print position to brain screen
  Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      bprint(0, "X: %8.2f", chassis.getPose().x);         // x
      bprint(1, "Y: %8.2f", chassis.getPose().y);         // y
      bprint(2, "Theta: %8.2f", chassis.getPose().theta); // heading
      // delay to save resources
      delay(20);
    }
  });
}

void opcontrol() {
  // loop forever
  while (true) {
    // get left y and right x positions
    int leftY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot
    chassis.arcade(leftY, rightX);

    // delay to save resources
    delay(25);
  }
}