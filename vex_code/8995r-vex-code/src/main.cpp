#include "main.h"
#include <cmath>

// controller 67
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup left_mg({-1, -2, 3}); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup right_mg({-8, 9, 10}); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu_sensor_left(4);
pros::Imu imu_sensor_right(7);

pros::Motor basket(5);
pros::Motor scoring(6);
pros::Motor pickup(11);
// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-17);

pros::adi::Pneumatics piston('A', false);
pros::adi::Pneumatics piston2('B', false); 

// PID constants
double kP_linear = 0.5;
double kI_linear = 0.0;    
double kD_linear = 0.0;

double kP_turn = 2.0;
double kI_turn = 0.0;
double kD_turn = 0.0;

const float wheel_diameter = 3.25; // in inches
const float wheel_circumference = wheel_diameter * M_PI; // in inches
const float degreesPerInch = 360.0 / wheel_circumference; // degrees per inch of travel

float intake_speed; 

// Global Variables
std::vector<double> global_position {0,0}; // x, y in inches 
float global_heading = 0; // in radians, facing east 

 // Utility functions
float wrap_angle(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return 1 * angle;
}
float calculate_error(float target, float current) {
    return target - current;
}

void odometry_task() {
    float lastForwardPos = 0;
    float lastHorizontalPos = 0;

    // Reset the rotation sensor at the start
    horizontalEnc.reset_position();

    while (true) {
        // 1. Update Heading (Radians)
        global_heading = wrap_angle(  (imu_sensor_left.get_rotation() + imu_sensor_right.get_rotation()) / 2 * M_PI / 180.0);
        
        // 2. Get current values
        // Forward position from drive encoders
        float currentForwardPos = (left_mg.get_position() + right_mg.get_position()) / 2.0;
        // Sideways position from the Rotation Sensor (convert centidegrees to degrees)
        float currentHorizontalPos = horizontalEnc.get_position() / 100.0;

        // 3. Calculate Deltas (change since last 10ms)
        float deltaForward = (currentForwardPos - lastForwardPos) / degreesPerInch;
        float deltaHorizontal = (currentHorizontalPos - lastHorizontalPos) / degreesPerInch;

        lastForwardPos = currentForwardPos;
        lastHorizontalPos = currentHorizontalPos;
        
        // 4. Transform Local Deltas to Global Deltas
        // This math rotates your local movement into the field's X and Y
        float deltaX = deltaForward * cos(global_heading) - deltaHorizontal * sin(global_heading);
        float deltaY = deltaForward * sin(global_heading) + deltaHorizontal * cos(global_heading);

        // 5. Update Global Position
        global_position[0] += deltaX;
        global_position[1] += deltaY;
        
        pros::delay(10); 
    }
}


 /**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    imu_sensor_left.reset();
    imu_sensor_right.reset();
    pros::delay(1000); // wait for imu to calibrate

    imu_sensor_left.set_heading(70); // set initial heading to 70 degrees
    imu_sensor_right.set_heading(70);

    // set initial position and heading
    global_position[0] = -48.83; 
    global_position[1] = 10.803; 
    global_heading = (7*M_PI)/18; // --> 70 degrees in radians 

    pros::Task odom_tracker(odometry_task);
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}


// PID CODE
void drive_to(float target_x, float target_y, float timeout, float speed_mult){
    float prev_dist_error = 0; 
    float integral = 0;
    int time_spent = 0;

    while(time_spent < timeout){
        float current_x = global_position[0];
        float current_y = global_position[1];

        float dist_error = sqrt(pow(target_x - current_x, 2) + pow(target_y - current_y, 2));
        float target_angle = atan2(target_y - current_y, target_x - current_x);
        
        // --- NEW REVERSE LOGIC ---
        if (speed_mult < 0) {
            target_angle += M_PI; // Aim the back of the robot at the target
        }

        float angle_error = target_angle - global_heading;

        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;
        // ----------------------------------------

        if(dist_error<3){
            integral += dist_error;
        } else{
            integral = 0;
        }

        float integral_limit = 20.0; // Max power the integral can contribute
        float integral_term = kI_linear * integral;

        // Constrain the integral term
        if (integral_term > integral_limit) integral_term = integral_limit;
        if (integral_term < -integral_limit) integral_term = -integral_limit;


        float derivative = dist_error - prev_dist_error; 
        float output = kP_linear * dist_error + integral_term + kD_linear * derivative;

        float drift_correction = angle_error * 10.0; // delete this if it causes unncecessary issues or basically set it to 0 

        // Apply speed_mult (it will be negative for reverse)
        left_mg.move((output * speed_mult) + drift_correction);
        right_mg.move((output * speed_mult) - drift_correction);

        if (dist_error < 0.75) break; 


        prev_dist_error = dist_error; 
        time_spent += 20;
        pros::delay(20);
    }
    left_mg.move(0);
    right_mg.move(0);
}
void turn_to(float target_angle, float timeout) {
    float prev_error = 0;
    float integral = 0;
    int time_spent = 0;

    while (time_spent < timeout) {
        // 1. Calculate Error
        float error = target_angle - global_heading;

        // 2. Wrap Angle (Shortest Path)
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;

        // 3. PID Math
        float derivative = error - prev_error;
        integral += error;
        float output = (kP_turn * error) + (kI_turn * integral) + (kD_turn * derivative);

        // 4. Motor Output (One side forward, one side back)
        left_mg.move(output);
        right_mg.move(-output);

        // 5. Exit Condition
        if (std::abs(error) < (1.0 * M_PI / 180.0)) break; // Stop if within 1 degree

        prev_error = error;
        time_spent += 20;
        pros::delay(20);
    }
    left_mg.brake();
    right_mg.brake();
}

void turn_to_point(float target_x, float target_y, float timeout) {
    float angle = atan2(target_y - global_position[1], target_x - global_position[0]);
    turn_to(angle, timeout);
}

// cordinates are based on path route from pathjerry.io 

void auton_skills(){
    // auton code skills here 
    global_position[0] = -49.028; 
    global_position[1] = 14.705;    
    global_heading = 0; // --> 70 degrees in radians
    imu_sensor_left.set_heading(0); // set initial heading to 0 degrees
    imu_sensor_right.set_heading(0);

    // move to loader 
    drive_to(-48.83, 47.441, 3000, 1.0);
    turn_to((3*M_PI)/2, 2000);
    drive_to(-59.345, 47.044, 3000, 1.0);
    // intake from loaders
    drive_to(-28.395, 46.846, 3000, -1.0);
    // outtake to long goal

    // move to other long goal 
    drive_to(-37.72, 46.648, 3000, 1.0);
    turn_to(M_PI, 2000);
    drive_to(-37.323, -47.592, 4000, 1.0);
    
}

void autonomous() {
    // move to field balls 
    drive_to(-26.411, 21.054, 3000, 1.0); 
    turn_to((16*M_PI)/9, 2000);

    // move to loader 
    drive_to(-48.632, 47.11, 3000, 1.0);
    turn_to((3*M_PI)/2, 2000);
    drive_to(-58.155, 46.714, 3000, 1.0);
    // intake from loaders 

    drive_to(-28.196, 47.64, 3000, -1.0);
    // outtake to long goal 
}

/**
 * Runs in driver control
 */
void opcontrol() {
    const int DEADBAND_THRESHOLD = 3; 
    // controller
    // loop to continuously update motors
    while (true) {
        double leftpower = 0;
        double rightpower = 0;
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        // move the chassis with curvature drive
        leftpower = leftY + rightX;
        rightpower = leftY - rightX;

        if (std::abs(leftpower) < 20) {
            leftpower = 0;
        }
        if (std::abs(rightpower) < 20) {
            rightpower = 0;
        }
            left_mg.move(leftpower);                          // Sets left motor voltage
            right_mg.move(rightpower);  

            // display position and heading on brain screen
            pros::lcd::print(0, "X: %.2f in", global_position[0]);
            pros::lcd::print(1, "Y: %.2f in", global_position[1]);
            pros::lcd::print(2, "Heading: %.2f deg", global_heading * 180.0 / M_PI);

        // intake control 
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            pickup.move_velocity(-200);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            pickup.move_velocity(200);
            scoring.move_velocity(200);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            pickup.move_velocity(-200);
            scoring.move_velocity(200);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            pickup.move_velocity(-200);
            scoring.move_velocity(-200);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            basket.move_velocity(200);
        } 
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            basket.move_velocity(-200);
        } else{
            basket.move_velocity(0);
            scoring.move_velocity(0);
            pickup.move_velocity(0);
        }

        // pneumatics
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            piston.toggle();
        } 

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
            piston2.toggle();
        }
        
        // delay to save resources
        pros::delay(10);
    }
}