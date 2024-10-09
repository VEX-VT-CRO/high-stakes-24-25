#include "subsystems/climb.hpp"

Climb::Climb(pros::MotorGroup& motors) : climbMotors{motors}
{
    climbMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    climbMotors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}

void Climb::moveClimb(int mV)
{
    climbMotors.move_voltage(mV);
}

void Climb::deployClimb_PB()
{
    climbMotors.move_absolute(4.6, 200);
    pros::delay(1380);
}

void Climb::deployClimb_J()
{
    climbMotors.move_absolute(2.9, 200);
    pros::delay(1000);
}