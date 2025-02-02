#include "subsystems/conveyorlift.hpp"


ConveyorLift::ConveyorLift(pros::MotorGroup& m, int extendedPosition) : motors{m}
{
    extendedPos = extendedPosition;
    position = ConveyorPosition::STOCK;

    motors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}

void ConveyorLift::moveTo(ConveyorPosition pos)
{
    position = pos;
    switch(position)
    {
    case STOCK:
        motors.move_absolute(0, 600);
        break;
    case ALLIANCE:
        motors.move_absolute(extendedPos * 0.2, 600);
        break;
    case SIDE:
        motors.move_absolute(extendedPos, 600);
        break;
    default:
        break;
    }
}


ConveyorPosition ConveyorLift::getPosition()
{
    return position;
}