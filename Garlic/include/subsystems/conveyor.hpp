#ifndef CONVEYOR_HPP
#define CONVEYOR_HPP

#include "pros/motor_group.hpp"

class Conveyor
{
    public:
        Conveyor(pros::MotorGroup& m);
        void spin(int mV);
        int first_ring_location = 200;
        int second_ring_location = 400;
        void move_to_location(int location);
    private:
        pros::MotorGroup& motors;
};

#endif