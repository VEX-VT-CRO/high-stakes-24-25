#include "subsystems/rollerIntake.hpp"

RollerIntake::RollerIntake(pros::MotorGroup& m) : motors{m}
{
    
}

void RollerIntake::spin(int mV)
{
    motors.move_voltage(mV);
}