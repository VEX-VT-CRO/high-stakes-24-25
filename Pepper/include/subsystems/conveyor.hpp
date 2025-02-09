#ifndef CONVEYOR_HPP
#define CONVEYOR_HPP

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

class Conveyor
{
    public:
        const int STANDARD_MV = 12000;
        Conveyor(pros::MotorGroup& m);
        void spin(int rpm);
    private:
        pros::MotorGroup& motors;
};

#endif