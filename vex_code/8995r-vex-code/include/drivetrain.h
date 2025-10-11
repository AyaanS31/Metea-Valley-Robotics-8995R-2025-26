#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "pros/motor_group.hpp"  // Only need MotorGroup here
#include <vector>  // For std::vector

class Drivetrain {
public:
    // Expose motor groups so external modules (like pid.cpp) can access them directly
    pros::MotorGroup left_motors;
    pros::MotorGroup right_motors;

    // Constructor: Pass vectors of port numbers (negative for reversed motors)
    // Constructor: Pass vectors of port numbers (negative for reversed motors)
    Drivetrain(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports);

    // Example methods you might want
    void tank_drive(int left_speed, int right_speed);  // For opcontrol with controller
    void move_distance(double distance_degrees, int velocity);  // For autonomous (relative move)
    void brake();  // Stop all motors
    // Add more for odometry/PID integration later, e.g., get_positions()
    // Return encoder positions (degrees) for odometry
    double get_left_position() const;
    double get_right_position() const;
};

// Global pointer (defined in a single translation unit, see src/main.cpp)
extern Drivetrain* drivetrain;

#endif