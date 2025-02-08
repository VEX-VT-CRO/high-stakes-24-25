#ifndef SIDESTAKES_HPP
#define SIDESTAKES_HPP

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

class SideStakes
{
    public:
        SideStakes(pros::MotorGroup& m);
        void spin(int mV);
        void moveToPosition(double position);
    private:
        pros::MotorGroup& motors;
};

#endif