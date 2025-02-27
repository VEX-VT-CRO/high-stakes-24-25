#include "subsystems/conveyorlift.hpp"
#include <cmath>
#include "pros/rtos.hpp"

ConveyorLift::ConveyorLift(pros::MotorGroup& m1, pros::MotorGroup& m2, pros::adi::DigitalOut& s1, pros::adi::DigitalOut& s2) : 
motors_front(m1), motors_back(m2), stopper_solenoid_front(s1), stopper_solenoid_back(s2),
open_stopper_front(false), open_stopper_back(false)
{
    position = STOCK;
    goingUp = true;
    motors_front.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    motors_back.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}

void ConveyorLift::openStopperFront()
{
    if (!open_stopper_front) {
        stopper_solenoid_front.set_value(1);
        open_stopper_front = true;
    } else {
        stopper_solenoid_front.set_value(0);
        open_stopper_front = false;
    }
}

void ConveyorLift::openStopperBack()
{
    if (!open_stopper_back) {
        stopper_solenoid_back.set_value(1);
        open_stopper_back = true;
    } else {
        stopper_solenoid_back.set_value(0);
        open_stopper_back = false;
    }
}

void ConveyorLift::moveTo(ConveyorPosition pos)
{
    position = pos;
    switch(position)
    {
        case MOGO:
            motors_front.move_absolute(0, 200);
            pros::delay(100);
            openStopperFront();
            motors_back.move_absolute(1.5, 200);
            pros::delay(100);
            motors_back.move_voltage(2000);
            openStopperBack();
            motors_back.move_voltage(0);
            break;
        case ALLIANCE:
            motors_front.move_absolute(9, 200);
            motors_back.move_absolute(9, 200);
            break;
        case SIDE:
            motors_front.move_absolute(0, 200);
            motors_back.move_absolute(0, 200);
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
            moveTo(MOGO);
            position = MOGO;
            break;
        case MOGO:
            moveTo(ALLIANCE);
            position = ALLIANCE;
            break;
        case ALLIANCE:
            if (goingUp)
            {
                moveTo(SIDE);
                position = SIDE;
                goingUp = false;
            }
            else
            {
                moveTo(MOGO);
                position = MOGO;
                goingUp = true;
            }
            break;
        case SIDE:
            moveTo(ALLIANCE);
            goingUp = false;
            position = ALLIANCE;
            break;
        default:
            break;
    }
}