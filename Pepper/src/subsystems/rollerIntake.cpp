#include "subsystems/rollerIntake.hpp"

RollerIntake::RollerIntake(pros::MotorGroup& m) : motors{m}
{
    
}

void RollerIntake::spin(int rpm)
{
    motors.move_velocity(rpm);
}