#ifndef ODOM_HPP
#define ODOM_HPP

#include "drivetrain.h"
// include pros RTOS header to access pros::Task
#include "pros/rtos.hpp"
#include <cmath>

const double track_width = 12233; 
const double strafe_center_offset = 3; // in inches
const double pod_tick_to_inch = (3.25*M_PI)/360.0; // wheel diameter 3.25 inches, 360 ticks per revolution

// Odometry task function
void odom_task(void* param);

#endif
