#include "odom.hpp"
#include "main.h"

// ODOMETRY TASK IMPLEMENTATION (Corrected Fusion Logic)

extern Pose current_pos;
extern pros::ADIEncoder encoder_vertical;
extern pros::ADIEncoder encoder_strafe;
extern Drivetrain* drivetrain;

void odom_task(void* param) {
    // --- 1. INITIALIZATION & RESET (Only runs once) ---
    
    // Reset ALL sensor encoders at the start
    encoder_vertical.reset(); 
    encoder_strafe.reset();
    drivetrain->left_motors.tare_position();  // Assuming Drivetrain class exposes the motor groups
    drivetrain->right_motors.tare_position(); // Assuming Drivetrain class exposes the motor groups

    // Initial Last Readings (To calculate the first delta correctly)
    long last_vertical_ticks = encoder_vertical.get_value();
    long last_strafe_ticks = encoder_strafe.get_value();
    double last_left_motor_pos = drivetrain->get_left_position(); 
    double last_right_motor_pos = drivetrain->get_right_position();

    // The continuous loop
    while (true) {
        // --- 2. READ CURRENT VALUES ---
        // Pods (ADI Encoders) - Raw Ticks
        double current_vertical_ticks = encoder_vertical.get_value();
        double current_strafe_ticks = encoder_strafe.get_value();
        
        // Motors (V5 Encoders) - Position in configured units (e.g., degrees)
        double current_left_motor_pos = drivetrain->get_left_position();
        double current_right_motor_pos = drivetrain->get_right_position();

        current_pos.theta = imu_sensor.get_rotation()*M_PI/180.0;

        // --- 3. CALCULATE DELTAS (Change in Distance/Position) ---
        // Convert the change in ticks/degrees to distance (inches)
        double delta_vertical = (current_vertical_ticks - last_vertical_ticks) * pod_tick_to_inch;
        double delta_strafe = (current_strafe_ticks - last_strafe_ticks) * pod_tick_to_inch;

        // --- 4. FUSION: CALCULATE POSE CHANGES ---
        
        // d_theta: Calculate rotation from the difference in motor movement
        
        // Average angle during the step

        // d_x_local: Forward movement from vertical pod
        double delta_x_local = delta_vertical;
        double delta_y_local = delta_strafe;

        // --- 5. CONVERT LOCAL TO GLOBAL MOVEMENT (Rotation Matrix) ---
        double d_x_global = delta_x_local * std::cos(current_pos.theta) - delta_y_local * std::sin(current_pos.theta);
        double d_y_global = delta_x_local * std::sin(current_pos.theta) + delta_y_local * std::cos(current_pos.theta);

        global_heading = current_pos.theta;
        
        // --- 6. UPDATE GLOBAL POSE ---
        current_pos.x += d_x_global;
        current_pos.y += d_y_global;

        // --- 7. UPDATE LAST READINGS ---
        last_vertical_ticks = current_vertical_ticks;
        last_strafe_ticks = current_strafe_ticks;


        pros::delay(5);
    }
};