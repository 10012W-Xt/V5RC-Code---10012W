#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <iostream>
#include <sys/signal.h>

/* ------------------------------- Controller ------------------------------- */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* --------------------------------- Motors --------------------------------- */
pros::MotorGroup right_drive({1, -11, 12}, pros::MotorGearset::blue);
pros::MotorGroup left_drive({-16, 21, -19}, pros::MotorGearset::blue);

pros::Motor intake_bottom(17);
pros::Motor intake_top(2);

int intake_bottom_speed;
int intake_top_speed;

/* --------------------------------- Sensors -------------------------------- */
pros::IMU inertial(5);
pros::Rotation vert(18);
pros::Rotation hor(14);

/* --------------------------------- Pistons -------------------------------- */
pros::adi::DigitalOut loader('H');
pros::adi::DigitalOut descorer('G');
pros::adi::DigitalOut centergoal('F');
pros::adi::DigitalOut trackwheel('E');

bool loader_down;
bool descorer_down;
bool centergoal_down;
bool trackwheel_down;

/* ----------------------------- Tracking Wheels ---------------------------- */
lemlib::TrackingWheel vert_wheel(&vert, lemlib::Omniwheel::NEW_2, 1.5);
lemlib::TrackingWheel hor_wheel(&hor, lemlib::Omniwheel::NEW_2, -3.25);

/* ---------------------------- Drivetrain Setup ---------------------------- */
lemlib::Drivetrain drivetrain(&left_drive, &right_drive, 10,
                              lemlib::Omniwheel::NEW_4, 500, 2);
lemlib::OdomSensors sensors(&vert_wheel, nullptr, &hor_wheel, nullptr,
                            &inertial);

/* ---------------------------------- PIDs ---------------------------------- */
lemlib::ControllerSettings
    lateral_controller(9,   // kP
                       0,   // kI
                       50,  // kD
                       3,   // antiwindup
                       1,   // small error range (in)
                       100, // small error range timeout (ms)
                       3,   // large error range (in)
                       500, // large error range timeout (ms)
                       5    // maximum acceleration
    );

lemlib::ControllerSettings
    angular_controller(2,   // kP
                       0,   // kI
                       26,  // kD
                       3,   // antiwindup
                       1,   // small error range (in)
                       100, // small error range timeout (ms)
                       3,   // large error range (in)
                       500, // large error range timeout
                       0    // maximum acceleration
    );

/* ----------------------------- Create Chassis ----------------------------- */
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors);

void checkIntakeStall(void *param) {
  int stallTimeMain = 0;
  int stallTimeL = 0;

  while (true) {
    // --- Get velocities ---
    int vMain = intake_bottom.get_actual_velocity();
    int vL = intake_top.get_actual_velocity();

    // --- Main Motor Stall Check ---
    if (intake_bottom_speed > 25 && vMain < 25)
      stallTimeMain += 10;
    else if (intake_bottom_speed < -25 && vMain > -25)
      stallTimeMain += 10;
    else
      stallTimeMain = 0;

    if (stallTimeMain >= 200) {
      intake_bottom.move(-intake_bottom_speed);
      pros::delay(150);
      intake_bottom.move(intake_top_speed);
      stallTimeMain = 0;
    }

    // --- Left Motor Stall Check ---
    if (intake_top_speed > 25 && vL < 25)
      stallTimeL += 10;
    else if (intake_bottom_speed - intake_top_speed < -25 && vL > -25)
      stallTimeL += 10;
    else
      stallTimeL = 0;

    if (stallTimeL >= 200) {
      intake_top.move(-intake_top_speed);
      pros::delay(150);
      intake_top.move(intake_top_speed);
      stallTimeL = 0;
    }

    pros::delay(10); // loop every 10ms
  }
}
void initialize() {
  /* ----------------------------- Motor Stopping -----------------------------
   */
  left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

  intake_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake_top.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  /* ---------------------------------- Setup ---------------------------------
   */
  pros::lcd::initialize();
  chassis.calibrate();
  intake_bottom.tare_position();

  /* ---------------------------- Position on Brain ---------------------------
   */
  pros::Task screen_task([&]() {
    while (true) {
      pros::lcd::print(0, "X: %f", chassis.getPose().x);
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);

      pros::delay(50);
    }
  });
}

void disabled() {}

void competition_initialize() {}

/* ------------------------------ My Functions ------------------------------ */

void print_coords() {
  double rounded_x;
  double rounded_y;
  double heading;

  while (true) {
    rounded_x = std::round(chassis.getPose().x * 100.0) / 100.0;
    rounded_y = std::round(chassis.getPose().y * 100.0) / 100.0;
    heading = std::round(chassis.getPose().theta * 100.0) / 100.0;

    std::cout << "x: " << rounded_x << ", y: " << rounded_y << std::endl;
    std::cout << "" << std::endl;

    pros::delay(500);
  }
}
void autonomous() {
  pros::Task intakeStallTask(checkIntakeStall, (void *)"",
                             "Intake Stall Protection");
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  intake_bottom.move(120);
  intake_top.move(-53);
  // Get Corner Blocks
  chassis.moveToPoint(0, 13.5, 1000);
  chassis.turnToPoint(-2, 19, 1000);
  // Collect the Blocks
  chassis.moveToPoint(-8, 28, 1000, {.maxSpeed = 60});
  chassis.waitUntilDone();
  loader.set_value(true);
  chassis.moveToPoint(-20, 37, 1000, {.maxSpeed = 80});
  chassis.waitUntil(10);
  loader.set_value(false);
  chassis.turnToPoint(-26, 40, 1000);
  chassis.moveToPoint(-27.5, 43.5, 1000, {.maxSpeed = 50});
  chassis.waitUntilDone();
  loader.set_value(true);
  chassis.moveToPoint(-5, 22, 1000, {.forwards = false, .maxSpeed = 80});
  chassis.turnToPoint(5, 39, 1000, {.forwards = false});
  chassis.moveToPoint(7, 44, 1000, {.forwards = false, .maxSpeed = 100});
  chassis.waitUntilDone();
  centergoal.set_value(true);
  pros::delay(800);
  centergoal.set_value(false);
  // Go to the Loader
  chassis.moveToPoint(-30, 4, 1000, {.maxSpeed = 110});
  chassis.turnToPoint(-35, -6, 1000);
  chassis.moveToPoint(-30, -28, 1000, {.maxSpeed = 60});
  chassis.moveToPoint(-30, -28, 1000, {.maxSpeed = 60});

  pros::delay(1000);
  // Go to the Long Goal
  chassis.moveToPoint(-31, 25, 1000, {.forwards = false, .maxSpeed = 127});
  chassis.waitUntilDone();
  // Dispence the blocks
  intake_bottom.move(127);
  intake_top.move(127);

  // For PID tuning
  // set position to x:0, y:0, heading:0
  // chassis.setPose(0, 0, 0);
  // turn to face heading 90 with a very long timeout
  // chassis.turnToHeading(90, 1000);
}

void opcontrol() {
  // Setup

  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  /* --------------------------- Drivetrain Control ---------------------------
   */
  // Get joystick positions
  while (true) {
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    int dead_zone = 15;
    // Deadzone
    if (abs(leftY) < dead_zone) {
      leftY = 0;
    }
    if (abs(rightX) < dead_zone) {
      rightX = 0;
    }

    // Move motors
    left_drive.move(leftY + rightX);
    right_drive.move(leftY - rightX);

    /* ----------------------------- Intake Control ----------------------------
     */
    // Intake
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {

      intake_bottom_speed = 127, intake_top_speed = -34;

      intake_bottom.move(intake_bottom_speed),
          intake_top.move(intake_top_speed);

      // Goals
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {

      intake_bottom_speed = 127, intake_top_speed = 127;

      intake_bottom.move(intake_bottom_speed),
          intake_top.move(intake_top_speed);

      // Outtake
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {

      intake_bottom_speed = -127;

      intake_bottom.move(intake_bottom_speed);

    } else {
      intake_bottom.move(0), intake_top.move(0);
    }

    /*---------------------------------- Pneumatics
     * ----------------------------------*/

    // Center Goal
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
      centergoal_down = !centergoal_down;
      centergoal.set_value(centergoal_down);

      // Tracking Wheel
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      trackwheel_down = !trackwheel_down;
      trackwheel.set_value(trackwheel_down);

      // Loader
    } else if (controller.get_digital_new_press(
                   pros::E_CONTROLLER_DIGITAL_L1)) {
      loader_down = !loader_down;
      loader.set_value(loader_down);

      // Descorer
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      descorer_down = !descorer_down;
      descorer.set_value(descorer_down);
    }
  }
  pros::delay(10);
}
