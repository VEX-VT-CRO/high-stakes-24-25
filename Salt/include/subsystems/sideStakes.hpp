#ifndef SIDESTAKES_HPP
#define SIDESTAKES_HPP

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

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
    void moveTo(SideStakesPosition pos, pros::Rotation &rotSensor);
    SideStakesPosition getPosition();
    const double STOCK_POS = 0;
    const double LOAD_POS = 100;
    const double SIDESTAKES_POS = 426;
private:
    pros::MotorGroup &motors;
    SideStakesPosition currentPosition;
};

#endif