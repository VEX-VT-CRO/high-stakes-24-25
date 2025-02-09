#ifndef CONVEYORLIFT_HPP
#define CONVEYORLIFT_HPP

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

enum ConveyorPosition
{
    STOCK,
    ALLIANCE,
    SIDE
};

class ConveyorLift
{
    public:
        const int STANDARD_MV = 12000;
        ConveyorLift(pros::MotorGroup& m, int extendedPosition);
        void moveTo(ConveyorPosition pos);
        ConveyorPosition getPosition();
        void toggle();
    private:
        pros::MotorGroup& motors;
        ConveyorPosition position;
        double extendedPos;
        bool goingUp;
};

#endif