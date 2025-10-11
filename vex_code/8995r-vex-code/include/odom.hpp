#ifndef ODOM_HPP
#define ODOM_HPP

#include "drivetrain.h"
// include pros RTOS header to access pros::Task
#include "pros/rtos.hpp"

class Odometry {
public:
    Odometry(Drivetrain* drivetrain, double wheel_diameter_mm, double track_width_mm);
    ~Odometry();

    // Start the odometry task (creates a PROS task internally)
    void start();
    void stop();

    // Getters for pose
    double get_x() const;
    double get_y() const;
    double get_theta() const; // degrees

private:
    static void task_entry(void*); // adapter for PROS task
    void run(); // instance task loop

    Drivetrain* drivetrain_;
    double x_; // mm
    double y_; // mm
    double theta_; // degrees
    double wheel_diameter_mm_;
    double track_width_mm_;

    volatile bool running_;
    pros::Task* task_handle_;
};

#endif
