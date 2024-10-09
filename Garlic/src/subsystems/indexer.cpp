#include "subsystems/indexer.hpp"
#include "pros/rtos.hpp"

// Use initializer list to initialize the solenoids and member variables
Indexer::Indexer(pros::adi::DigitalOut& s1, pros::adi::DigitalOut& s2, pros::adi::DigitalOut& s3, pros::adi::DigitalOut& s4, pros::adi::DigitalOut& s5) 
    : back_right_solenoid(s1), back_left_solenoid(s2), front_right_solenoid(s3), front_left_solenoid(s4), odometry_solenoid(s5)
{
   open_back_left = false;
   open_back_right = false;
   open_front_left = false;
   open_front_right = false;
}

void Indexer::openFront()
{
    openFrontLeft();
    openFrontRight();
}

void Indexer::openBack()
{
    openBackLeft();
    openBackRight();
}

void Indexer::openOdometry()
{
    // Placeholder for odometry solenoid control
}

void Indexer::openBackLeft()
{
    if (!open_back_left) {
        back_left_solenoid.set_value(1); // Activate solenoid
        open_back_left = true; 
    } else {
        back_left_solenoid.set_value(0); // Deactivate solenoid
        open_back_left = false;
    }
}

void Indexer::openBackRight()
{
    if (!open_back_right) {
        back_right_solenoid.set_value(1); // Activate solenoid
        open_back_right = true; 
    } else {
        back_right_solenoid.set_value(0); // Deactivate solenoid
        open_back_right = false;
    }
}

void Indexer::openFrontLeft()
{
    if (!open_front_left) {
        front_left_solenoid.set_value(1); // Activate solenoid
        open_front_left = true; 
    } else {
        front_left_solenoid.set_value(0); // Deactivate solenoid
        open_front_left = false;
    }
}

void Indexer::openFrontRight()
{
    if (!open_front_right) {
        front_right_solenoid.set_value(1); // Activate solenoid
        open_front_right = true; 
    } else {
        front_right_solenoid.set_value(0); // Deactivate solenoid
        open_front_right = false;
    }
}