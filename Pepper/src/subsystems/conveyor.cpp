#include "subsystems/conveyor.hpp"

Conveyor::Conveyor(pros::MotorGroup& m) : motors{m}
{
    
}

void Conveyor::spin(int rpm)
{
    motors.move_velocity(rpm);
}