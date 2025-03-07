#include "subsystems/indexer.hpp"
#include "pros/rtos.hpp"

Indexer::Indexer(pros::adi::DigitalOut& s1) 
    : clamp_solenoid(s1)
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

void Indexer::

openWing()
{
    if (!open_wing) {
        wing_solenoid.set_value(1); // Activate solenoid
        open_wing = true; 
    } else {
        wing_solenoid.set_value(0); // Deactivate solenoid
        open_wing = false;
    }
}