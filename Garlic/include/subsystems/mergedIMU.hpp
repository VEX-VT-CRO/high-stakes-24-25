#ifndef MERGEDIMU_HPP
#define MERGEDIMU_HPP

#include "pros/imu.hpp"

class MergedIMU : public pros::IMU {
public:
    MergedIMU(pros::IMU* top, pros::IMU* bottom, bool invertSecond)
        : pros::IMU(top->get_port()), t(top), b(bottom), inv(invertSecond) {}
    double get_heading() const override {
        double x = t->get_heading();
        double y = b->get_heading();
        if (inv) y = 360.0 - y;
        double r = (x + y) * 0.5;
        if (r < 0) r += 360.0;
        else if (r >= 360.0) r -= 360.0;
        return r;
    }
    std::int32_t tare_heading() const override {
        t->tare_heading();
        b->tare_heading();
        return 1;
    }
private:
    pros::IMU* t;
    pros::IMU* b;
    bool inv;
};

#endif