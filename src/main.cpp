

#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include <cmath>
#include <iostream>
#include <sys/_intsup.h>
#include <sys/signal.h>

/* ------------------------------- Controller ------------------------------- */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* --------------------------------- Motors --------------------------------- */
pros::MotorGroup right_drive({10, 3, -13}, pros::MotorGearset::blue);
pros::MotorGroup left_drive({17, -5, -16}, pros::MotorGearset::blue);

pros::Motor intake_bottom(9);
pros::Motor intake_top_left(20);
pros::Motor intake_top_right(1);

int intake_bottom_speed;
int intake_top_left_speed;
int intake_top_right_speed;

/* --------------------------------- Sensors -------------------------------- */ 
pros::IMU inertial(12);
pros::Optical colour(21);
pros::Rotation vert(2);
pros::Distance dist(19);

/* --------------------------------- Pistons -------------------------------- */
pros::adi::DigitalOut loader('H');
pros::adi::DigitalOut descorer('E');
pros::adi::DigitalOut longgoal ('F');
pros::adi::DigitalOut frontflap ('B');

bool loader_down;
bool descorer_down;
bool longgoal_down;
bool frontflap_down;

/* ----------------------------- Tracking Wheels ---------------------------- */
lemlib::TrackingWheel vert_wheel(&vert, lemlib::Omniwheel::NEW_2, 0);

/* ---------------------------- Drivetrain Setup ---------------------------- */
lemlib::Drivetrain drivetrain(&left_drive, &right_drive, 13.5,
                              lemlib::Omniwheel::NEW_4, 500, 8);
lemlib::OdomSensors sensors(&vert_wheel, nullptr, nullptr, nullptr,
                            &inertial);

/* ---------------------------------- PIDs ---------------------------------- */
lemlib::ControllerSettings
    lateral_controller(11,   // kP
                       0,   // kI
                       130,  // kD
                       3,   // antiwindup
                       1,   // small error range (in)
                       100, // small error range timeout (ms)
                       3,   // large error range (in)
                       500, // large error range timeout (ms)
                       20    // maximum acceleration
    );

lemlib::ControllerSettings
    angular_controller(3,   // kP
                       0,   // kI
                       20,  // kD
                       3,   // antiwindup
                       1,   // small error range (in)
                       100, // small error range timeout (ms)
                       3,   // large error range (in)
                       500, // large error range timeout (ms)
                       0    // maximum acceleration
    );

/* ----------------------------- Create Chassis ----------------------------- */

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);



// ------------------------------- Fix Drift ------------------------
void fixDrift () {

double errorX;
double errorY;
extern float x;
extern float y;
double dist_to_go_x;
double dist_to_go_y;

while (true) {
  // --- Distance from desired point ---
  dist_to_go_x = x - chassis.getPose().x;
  dist_to_go_y = y - chassis.getPose().y;
  // -- Calculate Distance from desired value --
  errorX = (chassis.getPose().theta/360) * 2*3.14159*dist_to_go_x;
  errorY = (chassis.getPose().theta/360) * 2*3.14159*dist_to_go_y;
   
  // Fix x errror
  if (errorX >= 5) {
    x = chassis.getPose().x + errorX;

  // Fix y error
  } else if (errorY >= 5) {
    y = chassis.getPose().y + errorY;

  }
  
  pros::delay(100);
}
// Delay for resoruces
pros::delay(10);

}

pros::Task* fixDrift_task = nullptr;

// Distance Sensors
void Distance () {
while (true) {
int distance = dist.get();
pros::lcd::print(0, "distance: %d cm", distance);
pros::lcd::register_btn1_cb(Distance);

    if (distance < 30) {
    
  }

pros::delay(10);
}}

pros::Task* distance_task = nullptr;

void initialize() {
  /* ----------------------------- Motor Stopping ---------------------------- */
   
  left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

  intake_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake_top_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake_top_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

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
    std::cout << "heading: " << heading << std::endl;

    pros::delay(500);
  }
}

/* ------------------------------- Autonmous -------------------------------- */

void autonomous() {
  //fixDrift_task = new pros::Task(fixDrift);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  intake_bottom.move(127),
    intake_top_left.move(-127),
    intake_top_right.move(127);
  chassis.moveToPoint(-1, 30, 600,{.maxSpeed = 127,. earlyExitRange = 2});

  pros::delay(350);
  loader.set_value(true);

  chassis.turnToPoint(31, 1, 800);
  chassis.moveToPoint(31, 13, 800,{.maxSpeed = 127,. earlyExitRange = 2});
  
  chassis.turnToPoint(37, -5, 800);
  chassis.moveToPoint(37, -5, 800);
  pros::delay(500);
  chassis.moveToPoint(24, 29, 800,{.forwards = false ,.maxSpeed = 80});
  chassis.waitUntilDone();              
  longgoal.set_value(true);
  



 

 

  //chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  // For PID tuning
  // set position to x:0, y:0, heading:0
  // turn to face heading 90 with a very long timeout
  //chassis.moveToPoint(0, 48 , 10000);
//chassis.turnToHeading(90, 10000);
 
}

void opcontrol() {
  // Setup

  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  /* --------------------------- Drivetrain Control --------------------------- */
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
    left_drive.move((leftY + rightX));
    right_drive.move((leftY - rightX));

    /* ----------------------------- Intake Control ---------------------------- */
    // Intake
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {

      intake_bottom_speed = 127,
      intake_top_left_speed = -127,
      intake_top_right_speed = 127;

    intake_bottom.move(intake_bottom_speed),
      intake_top_left.move(intake_top_left_speed),
      intake_top_right.move(intake_top_right_speed);

    // Long Goal
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {

    intake_bottom_speed = 127, 
      intake_top_left_speed = -127,
      intake_top_right_speed = 127;

      intake_bottom.move(intake_bottom_speed),
      intake_top_left.move(intake_top_left_speed),
      intake_top_right.move(intake_top_right_speed);

    // Center Goals
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {

      intake_bottom_speed = 127, 
      intake_top_left_speed = 12 ,
      intake_top_right_speed = 50;

      intake_bottom.move(intake_bottom_speed),
      intake_top_left.move(intake_top_left_speed),
      intake_top_right.move(intake_top_right_speed);

    // Outtake
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      
      intake_bottom_speed = -54,
      intake_top_left_speed = 14,
      intake_top_right_speed = -64;

      intake_bottom.move(intake_bottom_speed),
        intake_top_left.move(intake_top_left_speed),
        intake_top_right.move(intake_top_right_speed);

     // UnJam
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      
      intake_bottom_speed = -54,
      intake_top_left_speed = 52,
      intake_top_right_speed = -127;

      intake_bottom.move(intake_bottom_speed),
        intake_top_left.move(intake_top_left_speed),
        intake_top_right.move(intake_top_right_speed);

    } else {
    intake_bottom.move(0),
      intake_top_left.move(0),
      intake_top_right.move(0);
    }

    /*------------------------------- Pneumatics -------------------------------*/

    // Long Goal
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
      longgoal_down = !longgoal_down;
      longgoal.set_value(longgoal_down);
    } else if (controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_R2)) {
      longgoal_down = !longgoal_down;
      longgoal.set_value(longgoal_down);
      
    // Front flap 
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      frontflap_down = !frontflap_down;
      frontflap.set_value(frontflap_down);
    } else if (controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      frontflap_down = !frontflap_down;
      frontflap.set_value(frontflap_down);
    // Loader
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      loader_down = !loader_down;
      loader.set_value(loader_down);

    // Descorer
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      descorer_down = !descorer_down;
      descorer.set_value(descorer_down);
    }
  }
  pros::delay(10);
}
