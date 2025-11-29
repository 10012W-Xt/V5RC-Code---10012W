#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
//#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
// #include <stdexcept>
//#include <algorithm>
#include <sys/signal.h>
//#include <utility>



/* ------------------------------- Controller ------------------------------- */
pros::Controller controller(pros::E_CONTROLLER_MASTER);


/* --------------------------------- Motors --------------------------------- */
pros::MotorGroup left_drive({1, 16, -17}, pros::MotorGearset::blue);
pros::MotorGroup right_drive({-18, -20, 7}, pros::MotorGearset::blue);


pros::Motor intake_right_bottom(8);
pros::Motor intake_right_mid(9);
pros::Motor intake_right_top(10);
pros::Motor intake_left(11);


int intake_right_bottom_speed;
int intake_right_mid_speed;
int intake_right_top_speed;
int intake_left_speed;


/* --------------------------------- Sensors -------------------------------- */
pros::IMU inertial(13);
pros::Rotation vert(4);
pros::Rotation hor(5);
pros::Optical colour_sort(2);






/* --------------------------------- Pistons -------------------------------- */
pros::adi::DigitalOut loader('H');
pros::adi::DigitalOut descorer('G');

bool loader_down;
bool descorer_down;



/* ----------------------------- Tracking Wheels ---------------------------- */
lemlib::TrackingWheel vert_wheel(&vert, lemlib::Omniwheel::NEW_2, -1);
lemlib::TrackingWheel hor_wheel(&hor, lemlib::Omniwheel::NEW_2, 1);


/* ---------------------------- Drivetrain Setup ---------------------------- */
lemlib::Drivetrain drivetrain(
   &left_drive,
   &right_drive,
   10,
   lemlib::Omniwheel::NEW_325,
   500,
   8
);
lemlib::OdomSensors sensors(
   &vert_wheel,
   nullptr,
   &hor_wheel,
   nullptr,
   &inertial
);


/* ---------------------------------- PIDs ---------------------------------- */
lemlib::ControllerSettings lateral_controller(
   7, //kP
   0, //kI
   12, //kD
   3, // antiwindup
   1, // small error range (in)
   100, //small error range timeout (ms)
   3, //large error range (in)
   500, // large error range timeout (ms)
   10 // maximum acceleration
);


lemlib::ControllerSettings angular_controller(
   9, // kP
   0, // kI
   22, // kD
   3, // antiwindup
   1, // small error range (in)
   100, // small error range timeout (ms)
   3, // large error range (in)
   500, // large error range timeout
   7.6 // maximum acceleration
);


/* ----------------------------- Create Chassis ----------------------------- */
lemlib::Chassis chassis(
   drivetrain,
   lateral_controller,
   angular_controller,
   sensors
);


void initialize() {
   /* ----------------------------- Motor Stopping ----------------------------- */
   left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
   right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);


   intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   intake_right_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   intake_right_mid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   intake_right_top.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  
   /* ---------------------------------- Setup --------------------------------- */
   pros::lcd::initialize();
   chassis.calibrate();




   intake_right_bottom.tare_position();
   intake_right_mid.tare_position();
   intake_right_top.tare_position();
   intake_left.tare_position();


   /* ---------------------------- Position on Brain --------------------------- */
   pros::Task screen_task([&]() {
       while (true) {
           pros::lcd::print(0, "X: %f", chassis.getPose().x);
           pros::lcd::print(1, "Y: %f", chassis.getPose().y);
           pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);


           pros::delay(20);
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
   int arm_deg;


   while (true) {
       rounded_x = std::round(chassis.getPose().x * 100.0) / 100.0;
       rounded_y = std::round(chassis.getPose().y * 100.0) / 100.0;
       heading = std::round(chassis.getPose().theta * 100.0) / 100.0;
  
       std::cout << "x: " << rounded_x << ", y: " << rounded_y << std::endl;
       std::cout << "Heading:" << heading << ", Arm angle: " << arm_deg << std::endl;
       std::cout << "" << std::endl;


       pros::delay(500);
   }
}
void autonomous () {
chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
descorer_down = !descorer_down;
descorer.set_value(descorer_down);

intake_left_speed = -90,
intake_right_bottom_speed = 90,
intake_right_mid_speed = -90,
intake_right_top_speed = 90;
chassis.setPose(11,-15,10);

//Move to 3 midleft blocks

chassis.moveToPoint(18,30,1500,{.maxSpeed = 80});
//Go to Loader

chassis.turnToPoint(40.8,5.6,1000);
chassis.moveToPoint(40.8,5.6,1000,{.maxSpeed = 90});
loader_down = !loader_down;
loader.set_value(loader_down);
chassis.turnToPoint(44,5 , 1000);
chassis.turnToPoint(40,0,1000);
chassis.moveToPoint(44,-2,1000,{.maxSpeed = 90});
chassis.waitUntil(60);

//Bring blocks from loader to long goal
chassis.moveToPoint(40,15,1500,{.forwards = false, .maxSpeed = 100});
chassis.waitUntil(50);
chassis.waitUntilDone();
loader_down = !loader_down;
loader.set_value(loader_down);
chassis.turnToPoint(45,21,1500);
chassis.moveToPoint(45 ,25,1500);
chassis.waitUntil(10);
intake_left_speed = -90,
intake_right_bottom_speed = 0,
intake_right_mid_speed = 90,
intake_right_top_speed = -90;
pros::delay(50);
}

void opcontrol() {
   // Setup
 
   chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
 /* --------------------------- Drivetrain Control --------------------------- */
       // Get joystick positions
   while(true){
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


       /* ----------------------------- Intake Control ---------------------------- */
      // Top Bucket
       if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
           intake_right_bottom_speed = 127,
           intake_right_mid_speed = -127,
           intake_right_top_speed = 127,
           intake_left_speed = -127;


           intake_right_bottom.move(intake_right_bottom_speed),
           intake_right_mid.move(intake_right_mid_speed),
           intake_left.move(intake_left_speed),
           intake_right_top.move(intake_right_top_speed);
      
           pros::delay(20);


       // Outtake Low Center
       } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    
           intake_right_bottom_speed = -127,
           intake_right_top_speed = 127,
           intake_left_speed = 127,
           intake_right_mid_speed = 127;


           intake_left.move(intake_left_speed);
           intake_right_bottom.move(intake_right_bottom_speed),
           intake_right_mid.move(intake_right_mid_speed),
           intake_right_top.move(intake_right_top_speed);


       // Outtake Long Goal


       } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
           intake_right_top_speed = -127,
           intake_right_bottom_speed = 127,





           
           intake_left_speed = -127,
           intake_right_mid_speed = 127;


           intake_left.move(intake_left_speed),
           intake_right_mid.move(intake_right_mid_speed),
           intake_right_top.move(intake_right_top_speed);
        //
       }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {


           intake_right_top_speed = -127,
           intake_left_speed = -127,
           intake_right_mid_speed = 127;


           intake_left.move(intake_left_speed),
           intake_right_mid.move(intake_right_mid_speed),
           intake_right_top.move(intake_right_top_speed);


       } else {
           intake_left.move(0),
           intake_right_bottom.move(0),
           intake_right_mid.move(0),
           intake_right_top.move(0);
       }
      


      


   /*------------------------Pneumatics---------------------------*/


// Loader
if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
           loader_down = !loader_down;
           loader.set_value(loader_down);
}
//Descorer


if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
   descorer_down = !descorer_down;
   descorer.set_value(descorer_down);


}


   pros::delay(10);
}}
