#include "subsystems/conveyorlift.hpp"
#include <cmath>

ConveyorLift::ConveyorLift(pros::MotorGroup& m, int extendedPosition) : motors(m)
{
    extendedPos = extendedPosition;
    position = STOCK;
    goingUp = true;
    motors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}

void ConveyorLift::moveTo(ConveyorPosition pos)
{
    position = pos;
    switch(position)
    {
        case STOCK:
            motors.move_absolute(0, 200);
            break;
        case ALLIANCE:
            motors.move_absolute(extendedPos * 0.2, 200);
            break;
        case SIDE:
            motors.move_absolute(extendedPos, 200);
            break;
        default:
            break;
    }
}

ConveyorPosition ConveyorLift::getPosition()
{
    return position;
}

void ConveyorLift::toggle()
{
    switch (position)
    {
        case STOCK:
            moveTo(ALLIANCE);
            break;
        case ALLIANCE:
            if (goingUp)
            {
                moveTo(SIDE);
                goingUp = false;
            }
            else
            {
                moveTo(STOCK);
                goingUp = true;
            }
            break;
        case SIDE:
            moveTo(ALLIANCE);
            goingUp = false;
            break;
        default:
            break;
    }
}