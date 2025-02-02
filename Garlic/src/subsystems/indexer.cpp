#include "subsystems/indexer.hpp"
#include "pros/rtos.hpp"

// Constructor initializes solenoids
Indexer::Indexer(pros::adi::DigitalOut& s1, pros::adi::DigitalOut& s2, pros::adi::DigitalOut& s3, pros::adi::DigitalOut& s4) 
    : bumper_right_solenoid(s1), bumper_left_solenoid(s2), holder_right_solenoid(s3), holder_left_solenoid(s4),
      open_bumper_left(false), open_bumper_right(false), open_holder_left(false), open_holder_right(false)
{
}

void Indexer::openBumpers()
{
    openBumperLeft();
    openBumperRight();
}

void Indexer::openHolders()
{
    openHolderLeft();
    openHolderRight();
}

void Indexer::openBumperLeft()
{
    if (!open_bumper_left) {
        bumper_left_solenoid.set_value(1);
        open_bumper_left = true;
    } else {
        bumper_left_solenoid.set_value(0);
        open_bumper_left = false;
    }
}

void Indexer::openBumperRight()
{
    if (!open_bumper_right) {
        bumper_right_solenoid.set_value(1);
        open_bumper_right = true;
    } else {
        bumper_right_solenoid.set_value(0);
        open_bumper_right = false;
    }
}

void Indexer::openHolderLeft()
{
    if (!open_holder_left) {
        holder_left_solenoid.set_value(1);
        open_holder_left = true;
    } else {
        holder_left_solenoid.set_value(0);
        open_holder_left = false;
    }
}

void Indexer::openHolderRight()
{
    if (!open_holder_right) {
        holder_right_solenoid.set_value(1);
        open_holder_right = true;
    } else {
        holder_right_solenoid.set_value(0);
        open_holder_right = false;
    }
}