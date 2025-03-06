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
    ConveyorLift(pros::Motor &m1, pros::Motor &m2, pros::Motor &m3, pros::Motor &m4, pros::adi::DigitalOut &s1, pros::adi::DigitalOut &s2);
    void moveTo(ConveyorPosition pos);
    void toggle();
    void openStopperFront();
    void openStopperBack();
    bool open_stopper_front;
    bool open_stopper_back;
    bool goingUp;
    pros::Motor &motor_front_left;
    pros::Motor &motor_front_right;
    pros::Motor &motor_back_left;
    pros::Motor &motor_back_right;
    ConveyorPosition position;

private:
    // pros::MotorGroup &motors_front;
    pros::adi::DigitalOut &stopper_solenoid_front;
    pros::adi::DigitalOut &stopper_solenoid_back;
};

#endif