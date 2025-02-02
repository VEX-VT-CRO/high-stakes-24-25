#ifndef INDEXER_HPP
#define INDEXER_HPP

#include "pros/adi.hpp"

class Indexer
{
    public:
        Indexer(pros::adi::DigitalOut& br, pros::adi::DigitalOut& bl, pros::adi::DigitalOut& hr, pros::adi::DigitalOut& hl);

        void openBumpers();
        void openHolders();

    private:
        pros::adi::DigitalOut& bumper_right_solenoid;
        pros::adi::DigitalOut& bumper_left_solenoid;
        pros::adi::DigitalOut& holder_right_solenoid;
        pros::adi::DigitalOut& holder_left_solenoid;

        bool open_bumper_left;
        bool open_bumper_right;
        bool open_holder_left;
        bool open_holder_right;

        void openBumperLeft();
        void openBumperRight();
        void openHolderLeft();
        void openHolderRight();
};

#endif // INDEXER_HPP