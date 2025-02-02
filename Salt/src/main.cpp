#include "main.h"
#include "lemlib/api.hpp"

pros::Controller driver(pros::controller_id_e_t::E_CONTROLLER_MASTER);

pros::MotorGroup leftSide({1, 2, 15, -13}, pros::v5::MotorGears::blue);
pros::MotorGroup rightSide({-10, -9, -16, 18},pros::v5::MotorGearset::blue);

lemlib::Drivetrain drivetrain (&leftSide, &rightSide, 12, 3.25, 600, 2);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

void opcontrol()
{
    while(true)
    {
        int leftY = driver.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = driver.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_Y);

        chassis.tank(leftY, rightY);

        pros::delay(20);
    }

}

void autonomous()
{
    //Reset position to start
    chassis.setPose(-40, -40, 90);

    //Move to other side of field
    chassis.moveToPoint(40, -40, 5000);
    //chassis.moveToPose(40, -40, 90, 5000);
    
    //Start intake to get mobile goal

    //Turn to other corner to score
    chassis.turnToHeading(0, 2000);
    //chassis.turnToPoint(40, 40, 2000);
}

void krish()
{


}

void haanya()
{

}

void samhita()
{
    
}