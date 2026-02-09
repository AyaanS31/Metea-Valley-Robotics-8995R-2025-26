#include "main.h"
#include <cmath>

// controller 676767
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup left_mg({-11, -12, -13}, pros::MotorGear::blue);
pros::MotorGroup right_mg({15, 16, 17}, pros::MotorGearset::blue); 

pros::Imu imu_sensor(21);
pros::Distance distance_sensor_back(4);
pros::Distance distance_sensor_right(5);

pros::Motor intake(18);
pros::Motor lever(-14);

bool ArmUp = true;
bool LoaderUp = false;
bool WingUp = false;
bool LeverUp = false;
bool LeverMovingDown = false;
int leverStopCount = 0;


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 14, not reversed
pros::Rotation horizontalEnc(19); // horiztonal 
pros::Rotation verticalEnc(20); // vertical

pros::ADIDigitalOut LeverAir('H', false);
pros::ADIDigitalOut WingAir('A', false);
pros::ADIDigitalOut LoaderAir('G', false);
pros::ADIDigitalOut HoodAir('F', true);

double kP_linear = 22;
double kI_linear = 0;    
double kD_linear = 2 ;

double kP_turn = 5;
double kI_turn = 0.125;
double kD_turn = .32;

const float wheel_diameter = 3.25*0.99576271185; // in inches
const float wheel_circumference = wheel_diameter * M_PI; // in inches
const float degreesPerInch = 360.0 / wheel_circumference; // degrees per inch of travel
const float motor_degree_to_inch = 1.0 / degreesPerInch; // inches per degree of motor rotation
const double linear_error_threshold = .3; // inches
const double linear_settle_time = 3; // seconds
const double linear_integral_max = 50.0; // max integral term to prevent windup

const double angular_error_threshold = 1.0 * M_PI / 180.0; // radians (1 degree)
const double ANGULAR_SETTLE_TIME = 0.3; // seconds
const double angular_integral_max = 0.5; // max integral term to prevent wind

float intake_speed; 
// Global position variables
float global_x = 0.0; // can i change this initial value and it still works? 
float global_y = 0.0; 
float global_heading = 0.0; // in radians
float initialAngle;

 // Utility functions


float wrap_angle(float angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}


float calculate_error(float target, float current) {
    return target - current;
}


 /**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


void odometry_task() {
    double last_forward = (verticalEnc.get_position() / 100.0)
                          * motor_degree_to_inch;
    double lastStrafe = (horizontalEnc.get_position() / 100.0)
                         * motor_degree_to_inch;
    double last_heading = (imu_sensor.get_heading() * M_PI/180); 
    while (true) {
        // Forward distance (inches)
        double current_forward = (verticalEnc.get_position() / 100.0)
                          * motor_degree_to_inch;

        // Strafe distance (inches)
        double current_strafe = (horizontalEnc.get_position() / 100.0)
                         * motor_degree_to_inch;
        
        double current_heading = (imu_sensor.get_heading() * M_PI/180); 

        double dF = current_forward - last_forward;
        double dS = current_strafe - lastStrafe;
        double dTheta = wrap_angle(current_heading - last_heading); 

        double avgTheta = last_heading + (dTheta/2.0); 

        // Rotate into field frame (using radians btw)
        global_x += dF * cos(avgTheta) - dS * sin(avgTheta);
        global_y += dF * sin(avgTheta) + dS * cos(avgTheta);
        global_heading = wrap_angle(current_heading);

        last_forward = current_forward;
        lastStrafe = current_strafe;
        last_heading = current_heading;

        pros::delay(20);
    }
}

void disabled() {}

void competition_initialize() {}

void initialize() {
    pros::lcd::initialize();
    imu_sensor.reset();
    pros::Task odom_task(odometry_task);
}


// PID CODE
// Drives to XY coordinate using odometry. Turn to face target first with do_turn_global.
void drive(double target_x, double target_y, double drive_timeout, double speed, bool backward = false) {
    double integral = 0.0;
    double settle_timer = 0.0;

    std::int32_t last_time = pros::millis();
    double elapsed = 0.0;

    const double min_command = 8.0; // stiction compensation
    const double settle_time = 0.15; // seconds to consider settled
    
    // Direction: +1 forward, -1 backward
    double dir; 
    if(backward) dir = -1; else dir = 1; 

    // Initialize previous error
    double dx = target_x - global_x;
    double dy = target_y - global_y;
    double distance_error = std::sqrt(dx * dx + dy * dy);
    double prev_distance_error = distance_error; 

    while (elapsed < drive_timeout) {
        std::int32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0;
        if (dt <= 0.0) dt = 0.02;
        last_time = now;
        elapsed += dt;

        // Remaining distance to target (always positive)
        dx = target_x - global_x;
        dy = target_y - global_y;
        distance_error = std::sqrt(dx * dx + dy * dy);

        // PID (error always positive)
        double proportional = kP_linear * distance_error;

        if (distance_error < linear_error_threshold * 2.0) {
            integral += distance_error * dt;
        } else {
            integral = 0.0;
        }
        integral = std::clamp(integral, -linear_integral_max, linear_integral_max);
        double integral_term = kI_linear * integral;

        double derivative = (distance_error - prev_distance_error) / dt;
        double derivative_term = kD_linear * derivative;

        double linear_output = proportional + integral_term + derivative_term;

        // Stiction compensation
        if (linear_output > 0 && linear_output < min_command) {
            linear_output = min_command;
        }

        // Clamp to max speed
        linear_output = std::clamp(linear_output, 0.0, speed);

        // Apply direction for backward driving (dir results in either +1 or -1)
        int cmd = (int)std::round(dir * linear_output); 
        left_mg.move(cmd);
        right_mg.move(cmd);

        prev_distance_error = distance_error;

        // Settling check
        if (distance_error < linear_error_threshold) {
            settle_timer += dt;
            if (settle_timer >= settle_time) {
                break; // settled, exit loop
            }
        } else {
            settle_timer = 0.0;
        }

        pros::lcd::print(0, "X:%.1f Y:%.1f err:%.2f out:%d", global_x, global_y, distance_error, cmd);
        pros::delay(20);
    }

    left_mg.brake();
    right_mg.brake();
}


void do_turn(double target_angle, double turn_timeout, double max_speed) {
    double start_heading = wrap_angle(imu_sensor.get_rotation());
    double turn_error = target_angle; // initial error = target relative to start
    double prev_turn_error = turn_error;
    double integral = 0.0;

    double settle_timer = 0.0;
    bool is_settled = false;

    std::int32_t last_time = pros::millis();
    double elapsed = 0.0;

    while (elapsed < turn_timeout && !is_settled) {
        std::int32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0;
        if (dt <= 0.0) dt = 0.02;
        last_time = now;
        elapsed += dt;

        double current_heading = wrap_angle(imu_sensor.get_heading());
        turn_error = wrap_angle(target_angle - current_heading);


        // Deadzone to prevent vibration
        if (std::abs(turn_error) < wrap_angle(1.0)) turn_error = 0.0;

        // PID terms
        double proportional = kP_turn * turn_error;

        if (std::abs(turn_error) < 3.0) integral += turn_error * dt;
        else integral = 0.0;
        integral = std::clamp(integral, -20.0, 20.0);
        double integral_term = kI_turn * integral;

        double derivative = (turn_error - prev_turn_error) / dt;
        double derivative_term = kD_turn * derivative;

        double turn_output = proportional + integral_term + derivative_term;

        // Clamp output
        turn_output = std::clamp(turn_output, -max_speed, max_speed);

        left_mg.move(turn_output);
        right_mg.move(-turn_output);

        prev_turn_error = turn_error;

        if (std::abs(turn_error) < 1.0) settle_timer += dt;
        else settle_timer = 0.0;

        if (settle_timer >= ANGULAR_SETTLE_TIME) is_settled = true;

        pros::lcd::print(0, "Heading: %f | Error: %f", current_heading, turn_error);
        pros::delay(20);
    }

    left_mg.brake();
    right_mg.brake();
}

void do_turn_global(double target_global_heading_deg, double turn_timeout, double max_speed) {
    // Convert target to radians if your global_heading is in radians
    // Otherwise keep degrees
    double target_heading = wrap_angle(target_global_heading_deg); // degrees

    double current_heading = wrap_angle(imu_sensor.get_heading()); // degrees
    double turn_error = wrap_angle(target_heading - current_heading);
    double prev_turn_error = turn_error;
    double integral = 0.0;

    double settle_timer = 0.0;
    bool is_settled = false;

    std::int32_t last_time = pros::millis();
    double elapsed = 0.0;

    while (elapsed < turn_timeout && !is_settled) {
        std::int32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0;
        if (dt <= 0.0) dt = 0.02;
        last_time = now;
        elapsed += dt;

        current_heading = wrap_angle(imu_sensor.get_heading()); // read current heading
        turn_error = wrap_angle(target_heading - current_heading); // shortest-path error

        // Deadzone to prevent vibration
        if (std::abs(turn_error) < 0.3) turn_error = 0.0;

        // PID terms
        double proportional = kP_turn * turn_error;

        if (std::abs(turn_error) < 3.0) integral += turn_error * dt;
        else integral = 0.0;
        integral = std::clamp(integral, -20.0, 20.0);
        double integral_term = kI_turn * integral;

        double derivative = (turn_error - prev_turn_error) / dt;
        double derivative_term = kD_turn * derivative;

        double turn_output = proportional + integral_term + derivative_term;

        // Clamp output
        turn_output = std::clamp(turn_output, -max_speed, max_speed);

        left_mg.move(turn_output);
        right_mg.move(-turn_output);

        prev_turn_error = turn_error;

        if (std::abs(turn_error) < 1.0) settle_timer += dt;
        else settle_timer = 0.0;

        if (settle_timer >= ANGULAR_SETTLE_TIME) is_settled = true;

        pros::lcd::print(0, "Heading: %f | Error: %f", current_heading, turn_error);
        pros::delay(20);
    }
    left_mg.brake();
    right_mg.brake();
}

// cordinates are based on path route from pathjerry.io 

void auton_skills() {
    // ArmUpAir.set_value(true);
    // ArmUp = true;
    // WingAir.set_value(true);
    // WingUp = true;
    // linear_pid(58, 1.55, 80, false);
    // linear_pid(17, 0.67, 80, true);
    // hold.move(127);
    // do_turn_global(90, 0.7, 127); // deg, sec, 127
    // LoaderAir.set_value(true);
    // LoaderUp = true;
    // pros::delay(420);
    // linear_pid(15.5, 0.75, 35, false);
    //     linear_pid(1, 0.25, 127, true);
    //     linear_pid(4, 0.25, 127, false);
    //     pros::delay(500);
    //     linear_pid(1, 0.25, 127, true);
    //     linear_pid(4, 0.35, 127, false);
    //     linear_pid(3, 0.3, 127, false);
    //     pros::delay(400);
    // do_turn_global(90, 0.6, 127);
    // linear_pid(16, 0.7, 60, true);
    // linear_pid(6, 0.4, 70, false);
    // LoaderAir.set_value(false);
    // LoaderUp = false;

    // do_turn_global(0, 0.7, 127); // deg, sec, 127
    // hold.move(0);
    // linear_pid(14, 0.8, 80, false);
    // do_turn_global(-87, 1.1, 127); // deg, sec, 127
    // linear_pid(90, 2.15, 80, false);
    // do_turn_global(0, 1.1, 127); // deg, sec, 127

    // linear_pid(8, 0.5, 80, false);
    // pros::delay(250);



    // linear_pid(17, 0.75, 80, true);
    // do_turn_global(-90, 1.1, 127);
    // linear_pid(22, 0.70, 100, true);
    // linear_pid(6, 0.4, 35, true);
    // score.move(127);
    // pros::delay(760);
    //     score.move(-127);
    //     pros::delay(150);
    //     score.move(127);
    //     pros::delay(900);
    // do_turn_global(-90, 0.75, 127);
    // LoaderAir.set_value(true);
    // LoaderUp = true;
    // pros::delay(250);
    //     score.move(0);
    // hold.move(127);
    // linear_pid(38, 1.115, 50, false);
    //     linear_pid(1, 0.25, 100, true);
    //     linear_pid(4, 0.75, 100, false);
    //     linear_pid(6, 0.3, 100, false);
    //     linear_pid(5, 0.3, 80, false);
    //     pros::delay(300);
    // do_turn_global(-90, 0.6, 127);
    // linear_pid(30, 0.75, 75, true);
    // linear_pid(8, 0.15, 85, true);
    // do_turn_global(-90, 0.25, 127);
    // linear_pid(15, 0.25, 90, true);
    // hold.move(0);
    // score.move(92);
    // LoaderAir.set_value(false);
    // LoaderUp = false;
    // pros::delay(1250);
    // score.move(120);
    // pros::delay(450);
    // score.move(120);

    // // first 2 loaders done

    // linear_pid(12, 0.5, 127, false);
    // do_turn_global(182, 0.6, 127);
    // linear_pid(118, 2.0, 127, false);
    // linear_pid(18, 0.8, 127, true);
    // do_turn_global(-90, 0.6, 127);
    //     score.move(0);
    // hold.move(127);
    // LoaderAir.set_value(true);
    // LoaderUp = true;
    // pros::delay(500);
    //     score.move(0);
    // hold.move(127);
    // linear_pid(35, 0.95, 35, false);
    //     linear_pid(0.5, 0.25, 127, true);
    //     linear_pid(3, 0.25, 127, false);
    //     linear_pid(1, 0.25, 127, true);
    //     linear_pid(3, 0.25, 127, false);
    //     linear_pid(3, 0.25, 127, false);
    // do_turn_global(-90, 0.6, 127);
    // linear_pid(16, 0.75, 60, true);
    // linear_pid(6, 0.5, 60, false);
    // LoaderAir.set_value(false);
    // LoaderUp = false;

    // do_turn_global(180, 0.9, 127); // deg, sec, 127
    // hold.move(0);
    // linear_pid(14, 0.7, 90, false);
    // do_turn_global(98, 1.2, 127); // deg, sec, 127
    // linear_pid(90, 2.2, 90, false);
    // do_turn_global(180, 0.95, 127); // deg, sec, 127

    // linear_pid(8, 0.5, 90, false);
    // linear_pid(18.5, 0.8, 90, true);
    // do_turn(90, 0.8, 127);
    // linear_pid(22, 0.75, 100, true);
    // score.move(127);
    // pros::delay(750);
    //     score.move(-127);
    //     pros::delay(150);
    //     score.move(127);
    //     pros::delay(1000);

    // LoaderAir.set_value(true);
    // LoaderUp = true;
    // pros::delay(250);
    //     score.move(0);
    // hold.move(127);
    //     do_turn_global(90, 0.6, 127);
    // linear_pid(38.5, 1.1, 50, false);
    //     linear_pid(1, 0.25, 120, true);
    //     linear_pid(3, 0.25, 120, false);
    //     linear_pid(1, 0.25, 120, true);
    //     linear_pid(3, 0.125, 127, false);
    //     linear_pid(1, 0.125, 127, true);
    //     linear_pid(3, 0.25, 120, false);
    //     linear_pid(5, 0.25, 127, false);
    //     pros::delay(200);
    // do_turn_global(90.167, 0.7, 127);
    // linear_pid(32, 0.8, 80, true);
    // do_turn_global(90, 0.5, 127);
    // linear_pid(8, 0.25, 90, true);
    // hold.move(0);
    // score.move(102);
    // LoaderAir.set_value(false);
    // LoaderUp = false;
    // score.move(99);
    // linear_pid(8, 0.25, 127, true);
    // pros::delay(1470);

    // do_turn_global(57, 0.35, 127);
    // linear_pid(44, 0.75, 127, false);
    // do_turn_global(30, 0.72, 127);
    // linear_pid(5, 0.4, 127, false);
    // score.move(-127);
    // LoaderAir.set_value(true);
    // LoaderUp = true;
    // pros::delay(200);
    // hold.move(-127);
    // linear_pid(9, 0.25, 127, true);
    // linear_pid(34, 0.75, 127, false);
    // linear_pid(8, 0.25, 127, false);
    // LoaderAir.set_value(false);
    // LoaderUp = false;
}

void drive_to_point(double target_x, double target_y, double drive_timeout, double speed, bool backward = false) {
    // Negate target distance if driving backward

    // Tare motors
    left_mg.tare_position();
    right_mg.tare_position();
    pros::delay(50);

    double dx = pow(target_x - global_x, 2);
    double dy = pow(target_y - global_y, 2);

    double absolute_target = std::sqrt(dx + dy);
    double distance_error = absolute_target; // initial error
    double prev_distance_error = distance_error;

    double integral = 0.0;
    double integral_term = 0.0;
    double derivative = 0.0;
    double linear_output = 0.0;
    double settle_timer = 0.0;
    bool is_settled = false;

    std::int32_t last_time = pros::millis();
    double elapsed = 0.0;

    const double min_command = 8.0; // stiction

    while (elapsed < drive_timeout && !is_settled) {
        std::int32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0;
        if (dt <= 0.0) dt = 0.02;

        last_time = now;
        elapsed += dt;

        // Forward distance (inches i think)
        double current_y = (verticalEnc.get_position() / 100.0)
                          * motor_degree_to_inch;;

        // Strafe distance (inches i think)
        double current_x = (horizontalEnc.get_position() / 100.0)
                         * motor_degree_to_inch;; // ts should be updating everytime it moves if odom works perfectly. 

        current_x; 
        current_y; 

        double current_pos = std::sqrt(pow(target_x-current_x, 2) + pow(target_y-current_y,2));
        
        distance_error = absolute_target - current_pos;

        // PID
        double proportional = kP_linear * distance_error;

        if (std::abs(distance_error) < linear_error_threshold * 2.0) {
            integral += distance_error * dt;
        } else {
            integral = 0.0;
        }
        integral = std::clamp(integral, -linear_integral_max, linear_integral_max);
        integral_term = kI_linear * integral;

        derivative = (distance_error - prev_distance_error) / dt;
        double derivative_term = kD_linear * derivative;

        linear_output = proportional + integral_term + derivative_term;

        if (std::abs(linear_output) > 0 && std::abs(linear_output) < min_command) {
            linear_output = std::copysign(min_command, linear_output);
        }

        // Clamp to speed
        linear_output = std::clamp(linear_output, -speed, speed);

        int cmd = (int)std::round(linear_output);
        left_mg.move(cmd);
        right_mg.move(cmd);

        prev_distance_error = distance_error;

        // Settling
        if (std::abs(distance_error) < linear_error_threshold) {
            settle_timer += dt;
        } else {
            settle_timer = 0.0;
        }
        if (settle_timer >= linear_settle_time) {
            is_settled = true;
        }

        pros::lcd::print(0, "pos: %f err: %f P:%f I:%f D:%f out:%f dt:%f", distance_error, proportional, integral_term, derivative_term, linear_output, dt);

        pros::delay(20);
    }
    left_mg.brake();
    right_mg.brake();
    global_x = target_x; 
    global_y = target_y; 
}

void linear_pid(double target_distance, double drive_timeout, double speed, bool backward = false) {
    // Negate target distance if driving backward
    if (backward) target_distance = -target_distance;

    // Tare motors
    left_mg.tare_position();
    right_mg.tare_position();
    pros::delay(50);

    double k = 0.105; // error constant for 3.25 inch wheels
    double absolute_target = target_distance + (target_distance * k); // adjust target based on error constant
    double distance_error = absolute_target; // initial error
    double prev_distance_error = distance_error;

    double integral = 0.0;
    double integral_term = 0.0;
    double derivative = 0.0;
    double linear_output = 0.0;
    double settle_timer = 0.0;
    bool is_settled = false;

    std::int32_t last_time = pros::millis();
    double elapsed = 0.0;

    const double min_command = 8.0; // stiction

    while (elapsed < drive_timeout && !is_settled) {
        std::int32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0;
        if (dt <= 0.0) dt = 0.02;
        last_time = now;
        elapsed += dt;

        double currentPositionLeft = left_mg.get_position() * motor_degree_to_inch;
        double currentPositionRight = right_mg.get_position() * motor_degree_to_inch;
        double currentPosition = (currentPositionLeft + currentPositionRight) / 2.0;

        distance_error = absolute_target - currentPosition;

        // PID
        double proportional = kP_linear * distance_error;

        if (std::abs(distance_error) < linear_error_threshold * 2.0) {
            integral += distance_error * dt;
        } else {
            integral = 0.0;
        }
        integral = std::clamp(integral, -linear_integral_max, linear_integral_max);
        integral_term = kI_linear * integral;

        derivative = (distance_error - prev_distance_error) / dt;
        double derivative_term = kD_linear * derivative;

        linear_output = proportional + integral_term + derivative_term;

        // Stiction
        if (std::abs(linear_output) > 0 && std::abs(linear_output) < min_command) {
            linear_output = std::copysign(min_command, linear_output);
        }

        // Clamp to speed
        linear_output = std::clamp(linear_output, -speed, speed);

        int cmd = (int)std::round(linear_output);
        left_mg.move(cmd);
        right_mg.move(cmd);

        prev_distance_error = distance_error;

        // Settling
        if (std::abs(distance_error) < linear_error_threshold) {
            settle_timer += dt;
        } else {
            settle_timer = 0.0;
        }
        if (settle_timer >= linear_settle_time) {
            is_settled = true;
        }

        pros::lcd::print(0, "pos: %f err: %f P:%f I:%f D:%f out:%f dt:%f", currentPosition, distance_error, proportional, integral_term, derivative_term, linear_output, dt);

        pros::delay(20);
    }

    left_mg.brake();
    right_mg.brake();

}


void autonomous() {
    auton_skills();
}


// sauce
void opcontrol() {
    HoodAir.set_value(false); 
    while (true) {

        // Arcade drive
        int dir = controller.get_analog(ANALOG_LEFT_Y);
        int turn = controller.get_analog(ANALOG_LEFT_X);
        left_mg.move(dir + turn);
        right_mg.move(dir - turn);

        

        if (controller.get_digital(DIGITAL_L1) && !LeverUp) {
            intake.move(127);
        }

        else if (controller.get_digital(DIGITAL_L2)) {
            intake.move(-45);
        }

        else if (controller.get_digital_new_press(DIGITAL_R1)) {
            intake.move(127);
            pros::delay(50);    
            HoodAir.set_value(true);

            LeverUp = true;
            leverStopCount = 0;
            lever.move(68);

            intake.move(0);
        }

        else if (controller.get_digital_new_press(DIGITAL_R2)) {
            if (ArmUp) {
                LeverAir.set_value(false);
                pros::delay(20);
                ArmUp = false;
            } else {
                LeverAir.set_value(true);
                pros::delay(20);
                ArmUp = true;
            }
        }

        else if (controller.get_digital_new_press(DIGITAL_A)) {
            if (!WingUp) {
                WingAir.set_value(true);
                pros::delay(20);
                WingUp = true;
            } else {
                WingAir.set_value(false);
                pros::delay(20);
                WingUp = false;
            }
        }

        else if (controller.get_digital_new_press(DIGITAL_B)) {
            if (!LoaderUp) {
                LoaderAir.set_value(true);
                pros::delay(20);
                LoaderUp = true;
            } else {
                LoaderAir.set_value(false);
                pros::delay(20);
                LoaderUp = false;
            }
        }

        else if (LeverUp) {   // trying to move up
            if (abs(lever.get_actual_velocity()) < 10) {
                leverStopCount++;
            } else {
                leverStopCount = 0;
            }

            if (leverStopCount >= 3) {
                LeverUp = false;
                LeverMovingDown = true;
                leverStopCount = 0;

                HoodAir.set_value(false);
                lever.move(-127);
            }
        }

        else if (LeverMovingDown) { // trying to move down
            if (abs(lever.get_actual_velocity()) < 10) {
                leverStopCount++;
            } else {
                leverStopCount = 0;
            }

            if (leverStopCount >= 3) {
                LeverMovingDown = false;
                leverStopCount = 0;
                lever.move(0);
            }
        }

        else {
            intake.move(0);
        }

        pros::delay(25);
    }
}