#include "subsystems/sideStakes.hpp"

SideStakes::SideStakes(pros::MotorGroup& m, pros::Rotation& r) : motors{m}, rotation{r}
{
    
}


void SideStakes::spin(int mV)
{
    motors.move_voltage(mV);
}

void SideStakes::getPosition()
{
    rotation.get_position();
}

void SideStakes::moveToPosition(double position)
{
    motors.move_absolute(position, 100);
}