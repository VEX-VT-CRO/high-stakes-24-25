#include "subsystems/indexer.hpp"
#include "pros/rtos.hpp"

Indexer::Indexer(pros::adi::DigitalOut &s1, pros::adi::DigitalOut &s2)
    : holder_solenoid(s1), mover_solenoid(s2), open_holder(false), open_mover(false)
{
}

void Indexer::openHolder()
{
    if (!open_holder)
    {
        holder_solenoid.set_value(1);
        open_holder = true;
    }
    else
    {
        holder_solenoid.set_value(0);
        open_holder = false;
    }
}

void Indexer::openMover()
{
    if (!open_mover)
    {
        mover_solenoid.set_value(1);
        open_mover = true;
    }
    else
    {
        mover_solenoid.set_value(0);
        open_mover = false;
    }
}