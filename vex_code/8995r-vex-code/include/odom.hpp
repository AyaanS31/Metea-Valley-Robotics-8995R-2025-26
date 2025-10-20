#ifndef ODOM_HPP
#define ODOM_HPP

#include "drivetrain.h"
// include pros RTOS header to access pros::Task
#include "pros/rtos.hpp"
#include <cmath>

const double track_width = 10.5; // in inches 
const double strafe_center_offset = 5.75; // in inches
const double vertical_offset = 0.85; // in inches
const double pod_tick_to_inch = (2.0*M_PI)/360.0; // wheel diameter 2.0 inches, 360 ticks per revolution
const double motor_degree_to_inch = (3.25*M_PI)/360.0; // wheel diameter 3.25 inches, 360 degrees per revolution

// Odometry task function
void odom_task(void* param);

#endif
