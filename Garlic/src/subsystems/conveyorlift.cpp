#include "subsystems/conveyorlift.hpp"
#include <cmath>
#include "pros/rtos.hpp"

ConveyorLift::ConveyorLift(pros::Motor &m1, pros::Motor &m2, pros::Motor &m3, pros::Motor &m4, pros::adi::DigitalOut &s1, pros::adi::DigitalOut &s2) : motor_front_left(m1), motor_front_right(m2), motor_back_left(m3), motor_back_right(m4), stopper_solenoid_front(s1), stopper_solenoid_back(s2),
                                                                                                                                                       open_stopper_front(false), open_stopper_back(false)
{
    position = STOCK;
    goingUp = true;
    motor_front_left.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    motor_front_right.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    motor_back_left.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    motor_back_right.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    motor_front_left.tare_position();
    motor_front_right.tare_position();
    motor_back_left.tare_position();
    motor_back_right.tare_position();
}

void ConveyorLift::openStopperFront()
{
    if (!open_stopper_front)
    {
        stopper_solenoid_front.set_value(1);
        open_stopper_front = true;
    }
    else
    {
        stopper_solenoid_front.set_value(0);
        open_stopper_front = false;
    }
}

void ConveyorLift::openStopperBack()
{
    if (!open_stopper_back)
    {
        stopper_solenoid_back.set_value(1);
        open_stopper_back = true;
    }
    else
    {
        stopper_solenoid_back.set_value(0);
        open_stopper_back = false;
    }
}

void ConveyorLift::moveTo(ConveyorPosition pos)
{
    position = pos;
    switch (position)
    {
    case MOGO:
        if (goingUp)
        {
            openStopperFront();
            motor_back_left.move_absolute(150, 150);
            motor_back_right.move_absolute(150, 150);
            pros::delay(500);
            motor_back_left.move_voltage(6000);
            motor_back_right.move_voltage(6000);
            pros::delay(200);
            openStopperBack();
            motor_back_left.move_voltage(0);
            motor_back_right.move_voltage(0);
        }
        else
        {
            // openStopperBack();
            // openStopperFront();
            // pros::delay(100);
            // motors_back.move_absolute(2.5, 200);
            // motors_front.move_absolute(0, 200);
            // openStopperFront();
            // pros::delay(100);
            // motors_back.move_voltage(2000);
            // openStopperBack();
            // pros::delay(100);
            // motors_back.move_voltage(0);
        }
        break;
    case ALLIANCE:
        if (goingUp)
        {
            openStopperBack();
            openStopperFront();
            pros::delay(100);
            motor_back_left.move_relative(80, 100);
            motor_back_right.move_relative(80, 100);
            motor_front_left.move_relative(80, 100);
            motor_front_right.move_relative(80, 100);
            pros::delay(500);
            openStopperFront();
            motor_back_left.move_voltage(6000);
            motor_back_right.move_voltage(6000);
            pros::delay(200);
            openStopperBack();
            motor_back_left.move_voltage(0);
            motor_back_right.move_voltage(0);
        }
        else
        {
            // openStopperBack();
            // openStopperFront();
            // pros::delay(100);
            // motors_front.move_absolute(6, 200);
            // motors_back.move_absolute(8.5, 200);
            // openStopperFront();
            // pros::delay(100);
            // motors_back.move_voltage(2000);
            // openStopperBack();
            // pros::delay(100);
            // motors_back.move_voltage(0);
        }
        break;
    case SIDE:
        if (goingUp)
        {
            // openStopperBack();
            // openStopperFront();
            // pros::delay(100);
            // motors_front.move_absolute(10, 200);
            // motors_back.move_absolute(12.5, 200);
            // openStopperFront();
            // pros::delay(100);
            // motors_back.move_voltage(2000);
            // openStopperBack();
            // pros::delay(100);
            // motors_back.move_voltage(0);
        }
        break;
    default:
        break;
    }
}