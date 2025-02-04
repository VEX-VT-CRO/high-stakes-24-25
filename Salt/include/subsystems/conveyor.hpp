#ifndef CONVEYOR_HPP
#define CONVEYOR_HPP

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

class Conveyor
{
    public:
        Conveyor(pros::MotorGroup& m);
        void spin(int mV);
    private:
        pros::MotorGroup& motors;
};

#endif