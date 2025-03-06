#ifndef ROLLERINTAKE_HPP
#define ROLLERINTAKE_HPP

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

class RollerIntake
{
    public:
        RollerIntake(pros::MotorGroup& m);
        void spin(int rpm);
    private:
        pros::MotorGroup& motors;
};

#endif