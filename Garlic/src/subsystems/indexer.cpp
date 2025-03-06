#include "subsystems/indexer.hpp"
#include "pros/rtos.hpp"

Indexer::Indexer(pros::adi::DigitalOut& s1) 
    : holder_solenoid(s1),open_holder(false)
{

}


void Indexer::openHolder()
{
    if (!open_holder) {
        holder_solenoid.set_value(1);
        open_holder = true;
    } else {
        holder_solenoid.set_value(0);
        open_holder = false;
    }
}