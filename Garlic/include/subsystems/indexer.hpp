#ifndef INDEXER_HPP
#define INDEXER_HPP

#include "pros/adi.hpp"

class Indexer
{
public:
    Indexer(pros::adi::DigitalOut &hold, pros::adi::DigitalOut &move);
    void openHolder();
    void openMover();
    bool open_holder;
    bool open_mover;

private:
    pros::adi::DigitalOut &holder_solenoid;
    pros::adi::DigitalOut &mover_solenoid;
};

#endif