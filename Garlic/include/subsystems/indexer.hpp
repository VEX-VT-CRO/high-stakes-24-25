#ifndef INDEXER_HPP
#define INDEXER_HPP

#include "pros/adi.hpp"

class Indexer
{
    public:
        Indexer(pros::adi::DigitalOut& hold);
        void openHolder();
        bool open_holder;
    
    private:
        pros::adi::DigitalOut& holder_solenoid;
};

#endif