#ifndef ROLLERINTAKE_HPP
#define ROLLERINTAKE_HPP

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

class RollerIntake
{
    public:
        const int STANDARD_MV = 12000;
        RollerIntake(pros::MotorGroup& m);
        void spin(int mV);
    private:
        pros::MotorGroup& motors;
};

#endif