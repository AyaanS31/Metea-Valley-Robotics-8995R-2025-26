#include "main.h"
#include <chrono>
#include <cmath>
#include <vector>
#include "odom.hpp"
#include "pid.cpp"
// Assume this includes the Drivetrain class definition
#include "drivetrain.h" 
#include "pros/adi.h"
#include "pros/motors.h"

// --- GLOBAL STATE & SENSORS ---
Drivetrain* drivetrain = nullptr; // Global drivetrain pointer

Pose current_pos; // Global pose variable


// NOTE: Change these placeholder ports to match your hardware setup!
// Vertical Pod (Forward/Backward movement)
pros::Rotation rotation_sensor_vertical(14);
// Horizontal Pod (Strafe/Sideways movement)
pros::Rotation rotation_sensor_strafe(17);

// MOTOR CONSTANTS & PORTS

// Inertial Sensors
pros::Imu imu_sensor_left(4);
pros::Imu imu_sensor_right(7);


// Intake motors
pros::Motor intake_motor_1(5);
pros::Motor intake_motor_2(6);
pros::Motor intake_motor_3(11);

// Rotation Sensors 
pros::Rotation rotation_sensor(14);
pros::Rotation rotation_sensor_2(17);


/*
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

const double kP_linear = 0.75;   // Proportional gain for distance
const double kI_linear = 0.0;  // Integral gain for distance
const double kD_linear = 0.3;   // Derivative gain for distance

const double kP_turn = 59.4;   // Proportional gain for turn
const double kI_turn = 0.001;  // Integral gain for turn
const double kD_turn = 2.95;   // Derivative gain for turn

const double linear_error_threshold = 2.0; // inches
const double linear_settle_time = 3; // seconds
const double linear_integral_max = 50.0; // max integral term to prevent windup

const double angular_error_threshold = 1.0 * M_PI / 180.0; // radians (1 degree)
const double ANGULAR_SETTLE_TIME = 0.3; // seconds
const double angular_integral_max = 0.5; // max integral term to prevent wind

double wrap_angle(double angle){
    // Wrap angle to [-π, π] 
    while(angle > M_PI) angle -= 2 * M_PI;
    while(angle < -M_PI) angle += 2 * M_PI;
    return -1.0 * angle;
}

double calculate_error(double target, double current){
    return target - current;
}

// Angular PID Function 
void do_turn(double target_angle, double turn_timeout, double max_speed) {
    if (!drivetrain) return;

    // --- 1. INITIALIZATION AND ABSOLUTE TARGET CALCULATION ---
    
    // Get initial heading in radians
    double current_heading_rad = (imu_sensor_right.get_rotation() + imu_sensor_left.get_rotation()) / 2.0 * M_PI / 180.0;
    
    // Calculate the absolute angle the robot must reach.
    // Ensure the absolute target is wrapped for consistency, although the error calculation handles the wrap.
    double absolute_angle = wrap_angle(current_heading_rad + target_angle); 

    double turn_error = target_angle; // Initial error is the target displacement
    double prev_turn_error = turn_error;
    double integral = 0.0;
    
    double loop_counter = 0.0;
    double settle_timer = 0.0;
    bool is_settled = false;

    // --- 2. PID CONTROL LOOP ---
    // Loop until the safety timeout is reached OR the robot is settled
    while (loop_counter < turn_timeout && !is_settled) {
        
        // A. Get Current Heading
        current_heading_rad = (imu_sensor_right.get_rotation() + imu_sensor_left.get_rotation()) / 2.0 * M_PI / 180.0;

        // B. Error Calculation (using shortest path)
        // 1. Calculate raw difference
        double raw_error = absolute_angle - current_heading_rad;
        // 2. Wrap the error to [-pi, pi] to ensure shortest turn
        turn_error = wrap_angle(raw_error); 
        
        // C. PID Term Calculations
        
        // P - Proportional Term
        double proportional = kP_turn * turn_error;

        // I - Integral Term (with windup prevention)
        // Only integrate when error is small enough AND output isn't maxed out (for better anti-windup)
        if (std::abs(turn_error) < angular_error_threshold * 2.0) { 
            integral += turn_error * 0.02; // Assuming loop runs every 20 ms
        } else {
            integral = 0.0; // Reset if far from target
        }
        // Clamp the integral value
        if (integral > angular_integral_max) integral = angular_integral_max;
        if (integral < -angular_integral_max) integral = -angular_integral_max;
        double integral_term = kI_turn * integral;

        // D - Derivative Term (Time-Corrected)
        // The error used here must be the wrapped error!
        double derivative = (turn_error - prev_turn_error) / 0.02;
        double derivative_term = kD_turn * derivative;

        // Combine Terms and Clamp Output Speed
        double turn_output = proportional + integral_term + derivative_term;
        
        // Speed Clamp
        if (turn_output > max_speed) turn_output = max_speed;
        if (turn_output < -max_speed) turn_output = -max_speed;

        // Apply power (Left motor must be negative/opposite for turn)
        drivetrain->left_motors.move_velocity(-turn_output);
        drivetrain->right_motors.move_velocity(turn_output);

        // D. Update Tracking and Telemetry
        prev_turn_error = turn_error;
        loop_counter += 0.02; // Increment loop counter (assuming 20 ms loop)

        // E. Settling/Termination Condition
        if (std::abs(turn_error) < angular_error_threshold) {
            settle_timer += 0.02;
        } else {
            settle_timer = 0.0; // Reset timer if error is too large
        }
        if (settle_timer >= ANGULAR_SETTLE_TIME) {
            is_settled = true; // Exit loop after settling
        }
        
        pros::lcd::print(0, "Heading: %f | Error (Deg): %f", current_heading_rad * 180.0 / M_PI, turn_error * 180.0 / M_PI);
        pros::delay(20); // 20 ms delay
    }

    // --- 3. FINAL STOP ---
    drivetrain->left_motors.move_velocity(0);
    drivetrain->right_motors.move_velocity(0); 
}

// Linear PID Function

 void linear_pid(double target_distance, double drive_timeout, double speed){
    if (!drivetrain) return;


    if(drivetrain){

        double currentPositionLeft = drivetrain->left_motors.get_position();
        double currentPositionRight = drivetrain->right_motors.get_position();
        double currentPosition = (currentPositionLeft + currentPositionRight)/2.0; // average of left and right positions

        double absolute_target = currentPosition + target_distance; 
        double distance_error = target_distance; // initial error
        double prev_distance_error = distance_error; 
        
        double counter = 0.0;

        double integral = 0.0;
        double integral_term = 0.0;
        double derivative, linear_output;
        double settle_timer = 0.0;
        bool is_settled = false;

        while(counter < drive_timeout && !is_settled){
            currentPositionLeft = drivetrain->left_motors.get_position();
            currentPositionRight = drivetrain->right_motors.get_position();
            currentPosition = (currentPositionLeft + currentPositionRight)/2.0; // average of left and right positions

            distance_error = absolute_target - currentPosition; // distance needed to travel
            // Proportional Term
            double proportional = kP_linear * distance_error;

            // Integral Term
            if(std::abs(distance_error) < linear_error_threshold){ // only accumulate integral when within 5 inches of target
                integral += distance_error * 0.02; // assuming loop runs every 20 ms
                // Clamp integral to prevent windup
                if (integral > linear_integral_max) integral = linear_integral_max;
                if (integral < -linear_integral_max) integral = -linear_integral_max;
                integral_term = kI_linear * integral;
            } else {
                integral = 0.0; // reset integral if outside threshold
            }
            // Derivative Term

            derivative = (distance_error - prev_distance_error) / 0.02; // derivative term (rate of change)
            double derivative_term = kD_linear * derivative;

            // similar to angular PID, but for linear movement
            linear_output = proportional + integral_term + derivative_term;
            if(linear_output > speed) linear_output = speed;
            if(linear_output < -speed) linear_output = -speed;

            drivetrain->tank_drive(linear_output, linear_output);

            prev_distance_error = distance_error;

            if(std::abs(distance_error) < linear_error_threshold){
                settle_timer += 0.02;
            } else {
                settle_timer = 0.0;
            }
            if(settle_timer >= linear_settle_time){
                is_settled = true; // robot has settled at target
            }
            pros::lcd::print(0, "current position: %f \n error: %f", currentPosition, distance_error);
            counter += 0.02;

            pros::delay(20); // 20 ms delay
        }
    }
    drivetrain->tank_drive(0, 0); // stop the robot
}
 

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

// Deadband function to ignore small joystick inputs
int deadband(int value, int threshold) {
    if (std::abs(value) < threshold) {
        return 0;
    }
    return value;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	imu_sensor_right.reset(); // Reset IMU at start 
    imu_sensor_left.reset();
	drivetrain = new Drivetrain({1, 2, 3}, {-8, -9, -10});
	pros::Task odom(odom_task);

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


void autonomous() {
    linear_pid(10, 3, 100); // Move forward 10 inches, 3 second timeout, max speed 100
    pros::delay(1000);

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    const int DEADBAND_THRESHOLD = 5; // Analog values below 5 will be zeroed out

    while (true) {
        // Read raw analog values
        int left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int right_y = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // Apply deadband for smoother control
        int left_speed = deadband(left_y, DEADBAND_THRESHOLD);
        int right_speed = deadband(right_y, DEADBAND_THRESHOLD);

        // Tank Drive Control using the Drivetrain class method
        if (drivetrain) {
            drivetrain->tank_drive(left_speed, right_speed);
        }

        // Intake control
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_motor_1.move_velocity(200);
            intake_motor_2.move_velocity(200);
            intake_motor_3.move_velocity(200);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake_motor_1.move_velocity(-200);
            intake_motor_2.move_velocity(-200);
            intake_motor_3.move_velocity(-200);
        } else {
            intake_motor_1.move_velocity(0);
            intake_motor_2.move_velocity(0);
            intake_motor_3.move_velocity(0);
        }

        // Display odometry position on the controller 
        master.set_text(0, 0, "X:" + std::to_string(int(current_pos.x)) + " Y:" + std::to_string(int(current_pos.y)));

        pros::delay(20);
    }
}