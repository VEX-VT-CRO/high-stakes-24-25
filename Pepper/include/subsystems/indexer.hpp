#ifndef INDEXER_HPP
#define INDEXER_HPP

#include "pros/adi.hpp"

class Indexer
{
    public:
        // Constructor with updated type definitions
        Indexer(pros::adi::DigitalOut& r);
        
        // Public methods for controlling the solenoids
        void openClamp();
    
    private:
        // Member variables with updated type definitions
        pros::adi::DigitalOut& clamp_solenoid;

        // State tracking for the solenoids
        bool open_clamp;
};

#endif