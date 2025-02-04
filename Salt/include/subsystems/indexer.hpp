#ifndef INDEXER_HPP
#define INDEXER_HPP

#include "pros/adi.hpp"

class Indexer
{
    public:
        // Constructor with updated type definitions
        Indexer(pros::adi::DigitalOut& r, pros::adi::DigitalOut& l);
        
        // Public methods for controlling the solenoids
        void clamp();
        void openLeft();
        void openRight();
    
    private:
        // Member variables with updated type definitions
        pros::adi::DigitalOut& right_solenoid;
        pros::adi::DigitalOut& left_solenoid;

        // State tracking for the solenoids
        bool open_left;
        bool open_right;
};

#endif