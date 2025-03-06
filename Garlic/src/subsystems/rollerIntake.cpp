#include "subsystems/rollerIntake.hpp"

RollerIntake::RollerIntake(pros::Motor &m) : motor{m}
{
}

void RollerIntake::spin(int mV)
{
    motor.move_voltage(mV);
}