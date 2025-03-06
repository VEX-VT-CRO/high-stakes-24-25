#include "subsystems/conveyor.hpp"

Conveyor::Conveyor(pros::Motor &m) : motor{m}
{
}

void Conveyor::spin(int mV)
{
    motor.move_voltage(mV);
}

void Conveyor::move_to_location(int location)
{
    motor.move_absolute(location, 600);
}