#include "main.h"
#include "odom.hpp"

extern Pose current_pos;
extern pros::Rotation rotation_sensor_vertical;
extern pros::Rotation rotation_sensor_strafe;
extern Drivetrain* drivetrain;
extern pros::Imu imu_sensor_right; 
extern pros::Imu imu_sensor_left;

double average_heading() {
    double heading_right = imu_sensor_right.get_heading();
    double heading_left = imu_sensor_left.get_heading();
    // Handle wrap-around
    if (std::abs(heading_right - heading_left) > 180.0) {
        if (heading_right > heading_left) {
            heading_left += 360.0;
        } else {
            heading_right += 360.0;
        }
    }
    return (heading_right + heading_left) / 2.0;
}

void odom_task(void* param) {
    // --- 1. INITIALIZATION & RESET (Only runs once) ---
    rotation_sensor_vertical.reset_position();
    rotation_sensor_strafe.reset_position();
    drivetrain->left_motors.tare_position();  // Assuming Drivetrain class exposes the motor groups
    drivetrain->right_motors.tare_position(); // Assuming Drivetrain class exposes the motor groups

    // Initial Last Readings (To calculate the first delta correctly)
    double last_vertical_pos = rotation_sensor_vertical.get_position();
    double last_strafe_pos = rotation_sensor_strafe.get_position();
    double last_heading_rad = average_heading() * M_PI / 180.0; // Initial heading in radians

    // The continuous loop
    while (true) {
        // --- 2. READ CURRENT VALUES ---
        // Pods (ADI Encoders) - Raw Ticks
        double current_vertical_pos = rotation_sensor_vertical.get_position();
        double current_strafe_pos = rotation_sensor_strafe.get_position();

        double current_heading_rad = average_heading() * M_PI / 180.0; // Angle in Radians
        double delta_theta = current_heading_rad - last_heading_rad;  // Change in angle (Δθ)
        double theta_mid = last_heading_rad + (delta_theta / 2.0);   // Average angle during the step (θ_mid)
        current_pos.theta = current_heading_rad; // Update global heading

        // --- 3. CALCULATE DELTAS (Change in Distance/Position) ---
        // Convert the change in ticks/degrees to distance (inches)
        double delta_vertical = (current_vertical_pos - last_vertical_pos) * pod_tick_to_inch;
        double delta_strafe = (current_strafe_pos - last_strafe_pos) * pod_tick_to_inch;

        // --- 4. FUSION: CALCULATE POSE CHANGES ---
        
        // d_x_local: Forward movement from vertical pod
        double delta_x_local = delta_vertical;
        double delta_y_local = delta_strafe - (strafe_center_offset*pod_tick_to_inch);

        // --- 5. CONVERT LOCAL TO GLOBAL MOVEMENT (Rotation Matrix) ---
        double d_x_global = delta_x_local * std::cos(current_pos.theta) - delta_y_local * std::sin(current_pos.theta);
        double d_y_global = delta_x_local * std::sin(current_pos.theta) + delta_y_local * std::cos(current_pos.theta);

        double global_heading = current_pos.theta;

        // --- 6. UPDATE GLOBAL POSE ---
        current_pos.x += d_x_global;
        current_pos.y += d_y_global;

        // --- 7. UPDATE LAST READINGS ---
        last_vertical_pos = current_vertical_pos;
        last_strafe_pos = current_strafe_pos;


        pros::delay(10);
    }
};
