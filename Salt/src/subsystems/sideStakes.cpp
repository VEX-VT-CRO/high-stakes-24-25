#include "subsystems/sideStakes.hpp"

SideStakes::SideStakes(pros::MotorGroup& m) : motors{m}
{
    
}


void SideStakes::spin(int mV)
{
    motors.move_voltage(mV);
}

void SideStakes::moveToPosition(double position)
{
    motors.move_absolute(position, 100);
}