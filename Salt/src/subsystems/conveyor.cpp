#include "subsystems/conveyor.hpp"

Conveyor::Conveyor(pros::MotorGroup& m) : motors{m}
{
    
}

void Conveyor::spin(int mV)
{
    motors.move_voltage(mV);
}