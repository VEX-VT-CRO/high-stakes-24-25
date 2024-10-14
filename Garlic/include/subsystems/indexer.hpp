#ifndef INDEXER_HPP
#define INDEXER_HPP

#include "pros/adi.hpp"

class Indexer
{
    public:
        // Constructor with updated type definitions
        Indexer(pros::adi::DigitalOut& br, pros::adi::DigitalOut& bl, pros::adi::DigitalOut& fr, pros::adi::DigitalOut& fl, pros::adi::DigitalOut& odom);
        
        // Public methods for controlling the solenoids
        void openFront();
        void openBack();
        void openOdometry();

        void openFrontLeft();
        void openFrontRight();
        void openBackLeft();
        void openBackRight();
    
    private:
        // Member variables with updated type definitions
        pros::adi::DigitalOut& back_right_solenoid;
        pros::adi::DigitalOut& back_left_solenoid;
        pros::adi::DigitalOut& front_right_solenoid;
        pros::adi::DigitalOut& front_left_solenoid;
        pros::adi::DigitalOut& odometry_solenoid;

        // State tracking for the solenoids
        bool open_front_left;
        bool open_front_right;
        bool open_back_left;
        bool open_back_right;
};

#endif