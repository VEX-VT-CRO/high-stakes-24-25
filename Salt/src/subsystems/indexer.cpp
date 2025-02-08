#include "subsystems/indexer.hpp"
#include "pros/rtos.hpp"

Indexer::Indexer(pros::adi::DigitalOut& s1, pros::adi::DigitalOut& s2) 
    : right_solenoid(s1), left_solenoid(s2)
{
    open_left = false;
    open_right = false;

}

void Indexer::clamp()
{
    openLeft();
    openRight();
}



void Indexer::openLeft()
{
    if (!open_left) {
        left_solenoid.set_value(1); // Activate solenoid
        open_left = true; 
    } else {
        left_solenoid.set_value(0); // Deactivate solenoid
        open_left = false;
    }
}

void Indexer::openRight()
{
    if (!open_right) {
        right_solenoid.set_value(1); // Activate solenoid
        open_right = true; 
    } else {
        right_solenoid.set_value(0); // Deactivate solenoid
        open_right = false;
    }
}