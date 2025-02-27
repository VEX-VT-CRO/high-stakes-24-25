#ifndef CONVEYORLIFT_HPP
#define CONVEYORLIFT_HPP

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/adi.hpp"

enum ConveyorPosition
{
    STOCK,
    MOGO,
    ALLIANCE,
    SIDE
};

class ConveyorLift
{
    public:
        const int STANDARD_MV = 12000;
        ConveyorLift(pros::MotorGroup& m1, pros::MotorGroup& m2, pros::adi::DigitalOut& s1, pros::adi::DigitalOut& s2);
        void moveTo(ConveyorPosition pos);
        ConveyorPosition getPosition();
        void toggle();
        void openStopperFront();
        void openStopperBack();
        bool open_stopper_front;
        bool open_stopper_back;
    private:
        pros::MotorGroup& motors_front;
        pros::MotorGroup& motors_back;
        ConveyorPosition position;
        bool goingUp;
        pros::adi::DigitalOut& stopper_solenoid_front;
        pros::adi::DigitalOut& stopper_solenoid_back;
};

#endif