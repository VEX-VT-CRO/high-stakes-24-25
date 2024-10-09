#ifndef CLIMB_HPP
#define CLIMB_HPP

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

class Climb
{
    public:
        Climb(pros::MotorGroup& climbMotors);
        void moveClimb(int mV);
        void deployClimb_PB();
        void deployClimb_J();
    private:
        pros::MotorGroup& climbMotors;
};

#endif