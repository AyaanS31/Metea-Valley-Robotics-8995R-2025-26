// Odometry implementation using the Odometry class defined in include/odom.hpp
#include "odom.hpp"
#include <cmath>

Odometry::Odometry(Drivetrain* drivetrain, double wheel_diameter_mm, double track_width_mm)
    : drivetrain_(drivetrain), x_(0.0), y_(0.0), theta_(0.0),
      wheel_diameter_mm_(wheel_diameter_mm), track_width_mm_(track_width_mm),
      running_(false), task_handle_(nullptr) {}

Odometry::~Odometry() {
    stop();
}

void Odometry::start() {
    if (running_) return;
    running_ = true;
    task_handle_ = new pros::Task(&Odometry::task_entry, this, "odometry_task");
}

void Odometry::stop() {
    if (!running_) return;
    running_ = false;
    // Let the task exit on its own; delete handle
    if (task_handle_) {
        delete task_handle_;
        task_handle_ = nullptr;
    }
}

double Odometry::get_x() const { return x_; }
double Odometry::get_y() const { return y_; }
double Odometry::get_theta() const { return theta_; }

void Odometry::task_entry(void* ptr) {
    Odometry* self = static_cast<Odometry*>(ptr);
    if (self) self->run();
}

void Odometry::run() {
    const double wheel_circumference = wheel_diameter_mm_ * M_PI; 
    const double ticks_per_rev = 360.0; // encoder ticks per motor revolution

    double prev_left_ticks = 0.0; // initial left encoder ticks
    double prev_right_ticks = 0.0; // initial right encoder ticks 

    while (running_) {
        double left_ticks = 0.0;
        double right_ticks = 0.0;
        if (drivetrain_) {
            // Getting the position of the left and right encoders
            left_ticks = drivetrain_->get_left_position();
            right_ticks = drivetrain_->get_right_position();
        }
        // Calculate changes in encoder ticks since last update
        double delta_left = left_ticks - prev_left_ticks;
        double delta_right = right_ticks - prev_right_ticks;

        prev_left_ticks = left_ticks;
        prev_right_ticks = right_ticks;

        // Convert ticks to distance (mm)
        double left_distance = (delta_left / ticks_per_rev) * wheel_circumference;
        double right_distance = (delta_right / ticks_per_rev) * wheel_circumference;

        double delta_distance = (left_distance + right_distance) / 2.0;
        double delta_theta_rad = (right_distance - left_distance) / track_width_mm_; // radians

        // Update pose
        theta_ += delta_theta_rad * (180.0 / M_PI);
        // normalize
        if (theta_ >= 360.0) theta_ -= 360.0;
        if (theta_ < 0.0) theta_ += 360.0;

        double theta_rad = theta_ * (M_PI / 180.0); // in radians
        x_ += delta_distance * cos(theta_rad); // 
        y_ += delta_distance * sin(theta_rad);

        pros::delay(20);
    }
}

