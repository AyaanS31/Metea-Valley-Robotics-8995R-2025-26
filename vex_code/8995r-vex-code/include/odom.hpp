#ifndef ODOM_HPP
#define ODOM_HPP

#include "drivetrain.h"
// include pros RTOS header to access pros::Task
#include "pros/rtos.hpp"
#include <cmath>

const double track_width = 12233; 
const double strafe_center_offset = 3; // in inches
const double pod_tick_to_inch = (3.25*M_PI)/360.0; // wheel diameter 3.25 inches, 360 ticks per revolution
const double motor_tick_to_inch = (6.0*M_PI)/360.0; // wheel diameter 6.0 inches, 360 ticks per revolution

// Odometry task function
void odom_task(void* param);

// Internal variables to hold sensor readings from the previous loop iteration
double last_vertical_ticks = 0.0;
double last_strafe_ticks = 0.0;
double last_left_motor_pos = 0.0; 
double last_right_motor_pos = 0.0; 

#endif
