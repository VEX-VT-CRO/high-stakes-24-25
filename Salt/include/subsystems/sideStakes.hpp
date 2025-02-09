#ifndef SIDESTAKES_HPP
#define SIDESTAKES_HPP

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

enum class SideStakesPosition {
    STOCK,
    LOAD,
    SIDESTAKES
};

class SideStakes {
public:
    SideStakes(pros::MotorGroup &m);
    void spin(int mV);
    void moveToPosition(double pos);
    void moveTo(SideStakesPosition pos);
    SideStakesPosition getPosition();
private:
    pros::MotorGroup &motors;
    SideStakesPosition currentPosition;
    const double STOCK_POS = 0;
    const double LOAD_POS = 250;
    const double SIDESTAKES_POS = 1125;
};

#endif