#ifndef INDEXER_HPP
#define INDEXER_HPP

#include "pros/adi.hpp"

class Indexer
{
    public:
        // Constructor with updated type definitions
<<<<<<< HEAD
        Indexer(pros::adi::DigitalOut& solenoid);
        
        // Public methods for controlling the solenoids
        void release();
        void clamp();
        void toggle();
    
    private:
        // Member variables with updated type definitions
        pros::adi::DigitalOut& solenoid;

        // State tracking for the solenoids
        bool isClamped;
=======
        Indexer(pros::adi::DigitalOut& r, pros::adi::DigitalOut& l);
        
        // Public methods for controlling the solenoids
        void openClamp();
        void openWing();
    
    private:
        // Member variables with updated type definitions
        pros::adi::DigitalOut& clamp_solenoid;
        pros::adi::DigitalOut& wing_solenoid;

        // State tracking for the solenoids
        bool open_clamp;
        bool open_wing;
>>>>>>> 5c5038d6ae838a48e02e3a0077aeb0562073967b
};

#endif