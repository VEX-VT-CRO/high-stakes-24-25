#include "subsystems/indexer.hpp"
#include "pros/rtos.hpp"

<<<<<<< HEAD
// Use initializer list to initialize the solenoids and member variables
Indexer::Indexer(pros::adi::DigitalOut& s) : solenoid(s)
{
   isClamped = false;
   solenoid.set_value(0);
}

void Indexer::clamp()
{
    if(!isClamped)
    {
        solenoid.set_value(0);
        isClamped = true;
    }
}

void Indexer::release()
{
    if(isClamped)
    {
        solenoid.set_value(1);
        isClamped = false;
    }
}

void Indexer::toggle()
{
    if(isClamped)
    {
        solenoid.set_value(1);
        isClamped = false;
    }
    else
    {
        solenoid.set_value(0);
        isClamped = true;
=======
Indexer::Indexer(pros::adi::DigitalOut& s1, pros::adi::DigitalOut& s2) 
    : clamp_solenoid(s1), wing_solenoid(s2)
{
    open_clamp = false;
    open_wing = false;

}




void Indexer::openClamp()
{
    if (!open_clamp) {
        clamp_solenoid.set_value(1); // Activate solenoid
        open_clamp = true; 
    } else {
        clamp_solenoid.set_value(0); // Deactivate solenoid
        open_clamp = false;
    }
}

void Indexer::openWing()
{
    if (!open_wing) {
        wing_solenoid.set_value(1); // Activate solenoid
        open_wing = true; 
    } else {
        wing_solenoid.set_value(0); // Deactivate solenoid
        open_wing = false;
>>>>>>> 5c5038d6ae838a48e02e3a0077aeb0562073967b
    }
}