#ifndef INDEXER_HPP
#define INDEXER_HPP

#include "pros/adi.hpp"

class Indexer
{
    public:
        // Constructor with updated type definitions
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
};

#endif