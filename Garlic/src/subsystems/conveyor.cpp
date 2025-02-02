#include "subsystems/conveyor.hpp"

Conveyor::Conveyor(pros::MotorGroup& m) : motors{m}
{
    
}

void Conveyor::spin(int mV)
{
    motors.move_voltage(mV);
}

void Conveyor::move_to_location(int location)
{
    motors.move_absolute(location, 600);
}