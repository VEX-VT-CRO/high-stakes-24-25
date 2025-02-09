#include "subsystems/sideStakes.hpp"

SideStakes::SideStakes(pros::MotorGroup &m)
    : motors(m), currentPosition(SideStakesPosition::STOCK) {}

void SideStakes::spin(int mV) {
    motors.move_voltage(mV);
}

void SideStakes::moveToPosition(double pos) {
    motors.move_absolute(pos, 100);
}

void SideStakes::moveTo(SideStakesPosition pos) {
    currentPosition = pos;
    switch(pos) {
        case SideStakesPosition::STOCK:
            motors.move_absolute(STOCK_POS, 200);
            break;
        case SideStakesPosition::LOAD:
            motors.move_absolute(LOAD_POS, 200);
            break;
        case SideStakesPosition::SIDESTAKES:
            motors.move_absolute(SIDESTAKES_POS, 200);
            break;
        default:
            break;
    }
}

SideStakesPosition SideStakes::getPosition() {
    return currentPosition;
}