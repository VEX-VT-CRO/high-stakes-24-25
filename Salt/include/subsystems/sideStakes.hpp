#ifndef SIDESTAKES_HPP
#define SIDESTAKES_HPP

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

class SideStakes
{
    public:
        SideStakes(pros::MotorGroup& m, pros::Rotation& r);
        void spin(int mV);
        void getPosition();
        void moveToPosition(double position);
    private:
        pros::MotorGroup& motors;
        pros::Rotation& rotation;
};

#endif