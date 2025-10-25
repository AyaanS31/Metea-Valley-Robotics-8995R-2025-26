// #include "main.h"
// #include "odom.hpp"
// #include "main.cpp"
// #include "exception.h"

// extern Pose current_pos;
// extern pros::Rotation rotation_sensor_strafe;
// extern Drivetrain* drivetrain;
// extern pros::Imu imu_sensor_right;
// extern pros::Imu imu_sensor_left;

// // Average the two IMUs and return heading in degrees (handles wrap-around)
// double average_heading() {
//     double heading_right = imu_sensor_right.get_heading();
//     double heading_left = imu_sensor_left.get_heading();
//     // Handle wrap-around
//     if (std::abs(heading_right - heading_left) > 180.0) {
//         if (heading_right > heading_left) {
//             heading_left += 360.0;
//         } else {
//             heading_right += 360.0;
//         }
//     }
//     return (heading_right + heading_left) / 2.0;
// }

// void odom_task(void* param) {
//     // --- 1. INITIALIZATION & RESET (Only runs once) ---
//     rotation_sensor_strafe.reset_position();
//     // If drivetrain is null, bail out
//     if (!drivetrain) return;
//     drivetrain->left_motors.tare_position();
//     drivetrain->right_motors.tare_position();

//     // Basic sensor sanity checks: read once and validate
//     double left_deg = drivetrain->left_motors.get_position();
//     double right_deg = drivetrain->right_motors.get_position();
//     double initial_motor_avg_deg = (left_deg + right_deg) / 2.0;
//     double initial_strafe = rotation_sensor_strafe.get_position();
//     double initial_heading_deg = average_heading();
//     if (!std::isfinite(initial_motor_avg_deg) || !std::isfinite(initial_strafe) || !std::isfinite(initial_heading_deg)) {
//         // If sensors are invalid, abort the odom task
//         pros::lcd::print(0, "Odom init failure: bad sensor values");
//         return;
//     }

//     // Initial Last Readings (To calculate the first delta correctly)
//     double last_motor_avg_deg = initial_motor_avg_deg;
//     double last_strafe_pos = initial_strafe;
//     double last_heading_rad = initial_heading_deg * M_PI / 180.0; // Initial heading in radians

//     // The continuous loop
//     while (true) {
//         // --- 2. READ CURRENT VALUES ---
//         double cur_left_deg = drivetrain->left_motors.get_position();
//         double cur_right_deg = drivetrain->right_motors.get_position();
//         double cur_motor_avg_deg = (cur_left_deg + cur_right_deg) / 2.0;

//         // Lateral pod (ADI) - raw ticks/degrees
//         double cur_strafe_pos = rotation_sensor_strafe.get_position();

//         double cur_heading_rad = average_heading() * M_PI / 180.0; // radians

//         if (!std::isfinite(cur_motor_avg_deg) || !std::isfinite(cur_strafe_pos) || !std::isfinite(cur_heading_rad)) {
//             // Log and skip this iteration
//             pros::lcd::print(2, "Odom SensorError: non-finite reading");
//             pros::delay(100);
//             // attempt to resync last readings so we don't integrate bad data repeatedly
//             if (drivetrain) {
//                 last_motor_avg_deg = drivetrain->left_motors.get_position();
//                 last_motor_avg_deg = (last_motor_avg_deg + drivetrain->right_motors.get_position()) / 2.0;
//             }
//             last_strafe_pos = rotation_sensor_strafe.get_position();
//             last_heading_rad = average_heading() * M_PI / 180.0;
//             continue;
//         }

//         // Change in heading
//         double delta_theta = cur_heading_rad - last_heading_rad;
//         // normalize delta_theta to [-pi, pi]
//         while (delta_theta > M_PI) delta_theta -= 2.0 * M_PI;
//         while (delta_theta < -M_PI) delta_theta += 2.0 * M_PI;

//         // --- 3. CALCULATE DELTAS (Change in Distance/Position) ---
//         double delta_motor_deg = cur_motor_avg_deg - last_motor_avg_deg; // degrees
//         double delta_forward = delta_motor_deg * motor_degree_to_inch; // inches forward/back

//         double delta_strafe = (cur_strafe_pos - last_strafe_pos) * pod_tick_to_inch; // inches lateral

//         // Compensate lateral encoder for rotation: rotation causes apparent lateral travel = delta_theta * offset
//         double delta_y_local = delta_strafe - (delta_theta * strafe_center_offset);
//         double delta_x_local = delta_forward;

//         // Use mid-heading for better accuracy during the small motion
//         double theta_mid = last_heading_rad + (delta_theta / 2.0);

//         // --- 4. CONVERT LOCAL TO GLOBAL MOVEMENT (Rotation Matrix) ---
//         double d_x_global = delta_x_local * std::cos(theta_mid) - delta_y_local * std::sin(theta_mid);
//         double d_y_global = delta_x_local * std::sin(theta_mid) + delta_y_local * std::cos(theta_mid);

//         // --- 5. UPDATE GLOBAL POSE ---
//         current_pos.x += d_x_global;
//         current_pos.y += d_y_global;
//         current_pos.theta = cur_heading_rad;

//         // --- 6. UPDATE LAST READINGS ---
//         last_motor_avg_deg = cur_motor_avg_deg;
//         last_strafe_pos = cur_strafe_pos;
//         last_heading_rad = cur_heading_rad;

//         pros::delay(10);
//     }
// }