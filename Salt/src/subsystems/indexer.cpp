#include "subsystems/indexer.hpp"
#include "pros/rtos.hpp"

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
    }
}