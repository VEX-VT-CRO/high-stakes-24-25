#include "subsystems/sideStakes.hpp"
#include <stdlib.h>
#include <math.h>

SideStakes::SideStakes(pros::MotorGroup &m)
    : motors(m), currentPosition(SideStakesPosition::STOCK) {}

void SideStakes::spin(int mV) {
    motors.move_voltage(mV);
}

void SideStakes::moveToPosition(double pos) {
    motors.move_absolute(pos, 200);
}

SideStakesPosition SideStakes::getPosition() {
    return currentPosition;
}

void SideStakes::moveTo(SideStakesPosition pos, pros::Rotation &rotSensor) {
    currentPosition = pos;
    double desiredDegrees = 0;
    switch(pos) {
        case SideStakesPosition::STOCK:
            desiredDegrees = STOCK_POS;
            break;
        case SideStakesPosition::LOAD:
            desiredDegrees = LOAD_POS;
            break;
        case SideStakesPosition::SIDESTAKES:
            desiredDegrees = SIDESTAKES_POS;
            break;
        default:
            break;
    }
    double currentTicks = rotSensor.get_position();
    double desiredTicks = desiredDegrees * 100;
    double relativeTicks = desiredTicks - currentTicks;
    motors.move_relative(relativeTicks/100, 200);
}