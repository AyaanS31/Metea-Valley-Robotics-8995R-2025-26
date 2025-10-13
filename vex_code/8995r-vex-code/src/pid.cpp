#include "main.h"
#include <cmath>
#include "drivetrain.h"
using namespace std;
// PID instances for linear and angular control
// Random values assigned for now 
const double kP_linear = 0.75;   // Proportional gain for distance
const double kI_linear = 0.0;  // Integral gain for distance
const double kD_linear = 0.3;   // Derivative gain for distance

const double kP_turn = 59.4;   // Proportional gain for turn
const double kI_turn = 0.001;  // Integral gain for turn
const double kD_turn = 2.95;   // Derivative gain for turn

const double degreesPerInch = 360.0 / (3.25 * M_PI); // 3.25 inch wheels

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
void do_turn(double angle, double turn_timeout){
    if (drivetrain) {
        drivetrain->left_motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        drivetrain->right_motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        double turn_error = angle - global_heading; 
        double counter = 0.0;
        double prev_turn = 0.0; // persist previous error across iterations for derivative
        while(counter < turn_timeout){
            // update the shared global_heading (do NOT redeclare a new local variable)
            turn_error = angle - global_heading;
            double turn_derivative = (turn_error - prev_turn);
            double turn_output = (kP_turn * turn_error) + (kI_turn * 0.0) + (kD_turn * turn_derivative); // total error output

            // Using turn output to set motor velocities for turning
            // Positive turn_output means we need to turn clockwise, so left motors go forward, right motors go backward
            drivetrain->left_motors.move_velocity(-turn_output);
            drivetrain->right_motors.move_velocity(turn_output);
            pros::lcd::print(0, "heading: %f \n turn_error: %f", global_heading, turn_error * 180.0 / M_PI);
            
            prev_turn = turn_error;
            counter += 0.02;
            pros::delay(20); // 20 ms delay
        }
    }
}

// Linear PID Function
void linear_pid(double target_position, double drive_timeout, double speed){
    if(drivetrain){
        drivetrain->left_motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        drivetrain->right_motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        double target_x = target_position;
        double x;
        x = global_position[0];

        double distance_to_target = target_x;
        double turn_error, distance_error, heading_error;
        double counter = 0.0;

        double prev_distance = 0.0, integral = 0.0;
        double derivative, linear_output, currentPositionLeft, currentPositionRight, currentPosition;
        double target_degrees = distance_to_target * degreesPerInch; // Convert inches to degrees
        double dx, dy; 

        while(counter < drive_timeout){
            currentPositionLeft = drivetrain->left_motors.get_position();
            currentPositionRight = drivetrain->right_motors.get_position();
            currentPosition = (currentPositionLeft + currentPositionRight)/2.0; // average of left and right positions

            distance_error = target_degrees - currentPosition; // distance needed to travel
            derivative = (distance_error - prev_distance);
            integral += distance_error;

            // similar to angular PID, but for linear movement
            linear_output = (kP_linear * distance_error) + (kI_linear * integral) + (kD_linear * derivative);
            if(linear_output > speed) linear_output = speed;
            if(linear_output < -speed) linear_output = -speed;

            drivetrain->tank_drive(linear_output, linear_output);

            prev_distance = distance_error;
            pros::lcd::print(0, "current position: %f \n error: %f", currentPosition / degreesPerInch, distance_error);
            dx = double(cos(global_heading) * currentPosition / degreesPerInch); // converted degrees back into inches,
            dy = double(sin(global_heading) * currentPosition / degreesPerInch); // since global position is based on inches
            global_position[0] += dx;
            global_position[1] += dy;
            counter += 0.02;

            pros::delay(20); // 20 ms delay
        }
    }
}