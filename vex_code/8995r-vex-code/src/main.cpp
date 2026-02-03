#include "main.h"
#include <cmath>


// controller 6767
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup left_mg({-11, -13, -16}, pros::MotorGear::blue);
pros::MotorGroup right_mg({10, 18, 19}, pros::MotorGearset::blue); 

pros::Imu imu_sensor(17);
pros::Distance distance_sensor_back(4);
pros::Distance distance_sensor_right(5);

pros::MotorGroup score({3,8,9});
pros::MotorGroup hold({3,-8,9});
pros::MotorGroup pre({9});

pros::Motor intakePre(9);
pros::Motor intakeMain(3);
pros::Motor intakeHood(8);

bool ArmUp = false;
bool ArmDown = false;
bool LoaderUp = false;
bool WingUp = false;


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 14, not reversed
pros::Rotation horizontalEnc(14); // horiztonal 
pros::Rotation verticalEnc(15); // vertical tracking wheel encoder. Rotation sensor, port 15, not reversed

pros::ADIDigitalOut ArmUpAir('F', false);
pros::ADIDigitalOut ArmDownAir('G', false);
pros::ADIDigitalOut LoaderAir('H', false);
pros::ADIDigitalOut WingAir('E', false);

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
std::vector<float> global_position = {0.0, 0.0}; // X, Y in inches
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
    double lastForward = 0;
    double lastStrafe = 0;

    verticalEnc.reset_position();
    horizontalEnc.reset_position();

    while (true) {
        // Heading from IMU (degrees)
        double headingDeg = imu_sensor.get_heading();
        global_heading = wrap_angle(headingDeg);
        // Convert to radians for trig
        double headingRad = global_heading * M_PI / 180.0;

        // Forward distance (inches)
        double forward = (verticalEnc.get_position() / 100.0)
                          * motor_degree_to_inch;

        // Strafe distance (inches)
        double strafe = (horizontalEnc.get_position() / 100.0)
                         * motor_degree_to_inch;

        double dF = forward - lastForward;
        double dS = strafe - lastStrafe;

        lastForward = forward;
        lastStrafe = strafe;

        // Rotate into field frame (using radians)
        double dx = dF * cos(headingRad) - dS * sin(headingRad);
        double dy = dF * sin(headingRad) + dS * cos(headingRad);

        global_position[0] += dx;
        global_position[1] += dy;

        pros::delay(10);
    }
}

void disabled() {}

void competition_initialize() {}

void initialize() {
    pros::lcd::initialize();
    imu_sensor.reset();
    pros::Task odom_task(odometry_task);
    double leftSevenWingAngle = 25;
    double leftSevenWingX = 0;
    double leftSevenWingY = 0;

    int allOther = 0;

    imu_sensor.set_heading(allOther);
    initialAngle = allOther;
    global_position[0] = 0;
    global_position[1] = 0;

}


// PID CODE
// Drives to XY coordinate using odometry. Turn to face target first with do_turn_global.
void drive_to_point(double target_x, double target_y, double drive_timeout, double speed, bool backward = false) {
    double distance_error = 0.0;
    double prev_distance_error = 0.0;
    double integral = 0.0;
    double settle_timer = 0.0;

    std::int32_t last_time = pros::millis();
    double elapsed = 0.0;

    const double min_command = 8.0; // stiction compensation
    const double settle_time = 0.15; // seconds to consider settled
    
    // Direction: +1 forward, -1 backward
    double dir = if(backward) -1.0; else 1.0;

    // Initialize previous error
    double dx = target_x - global_position[0];
    double dy = target_y - global_position[1];
    prev_distance_error = std::sqrt(dx * dx + dy * dy);

    while (elapsed < drive_timeout) {
        std::int32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0;
        if (dt <= 0.0) dt = 0.02;
        last_time = now;
        elapsed += dt;

        // Remaining distance to target (always positive)
        dx = target_x - global_position[0];
        dy = target_y - global_position[1];
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

        pros::lcd::print(0, "X:%.1f Y:%.1f err:%.2f out:%d", global_position[0], global_position[1], distance_error, cmd);
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
    ArmUpAir.set_value(true);
    ArmUp = true;
    WingAir.set_value(true);
    WingUp = true;
    linear_pid(58, 1.55, 80, false);
    linear_pid(17, 0.67, 80, true);
    hold.move(127);
    do_turn_global(90, 0.7, 127); // deg, sec, 127
    LoaderAir.set_value(true);
    LoaderUp = true;
    pros::delay(420);
    linear_pid(15.5, 0.75, 35, false);
        linear_pid(1, 0.25, 127, true);
        linear_pid(4, 0.25, 127, false);
        pros::delay(500);
        linear_pid(1, 0.25, 127, true);
        linear_pid(4, 0.35, 127, false);
        linear_pid(3, 0.3, 127, false);
        pros::delay(400);
    do_turn_global(90, 0.6, 127);
    linear_pid(16, 0.7, 60, true);
    linear_pid(6, 0.4, 70, false);
    LoaderAir.set_value(false);
    LoaderUp = false;

    do_turn_global(0, 0.7, 127); // deg, sec, 127
    hold.move(0);
    linear_pid(14, 0.8, 80, false);
    do_turn_global(-87, 1.1, 127); // deg, sec, 127
    linear_pid(90, 2.15, 80, false);
    do_turn_global(0, 1.1, 127); // deg, sec, 127

    linear_pid(8, 0.5, 80, false);
    pros::delay(250);



    linear_pid(17, 0.75, 80, true);
    do_turn_global(-90, 1.1, 127);
    linear_pid(22, 0.70, 100, true);
    linear_pid(6, 0.4, 35, true);
    score.move(127);
    pros::delay(760);
        score.move(-127);
        pros::delay(150);
        score.move(127);
        pros::delay(900);
    do_turn_global(-90, 0.75, 127);
    LoaderAir.set_value(true);
    LoaderUp = true;
    pros::delay(250);
        score.move(0);
    hold.move(127);
    linear_pid(38, 1.115, 50, false);
        linear_pid(1, 0.25, 100, true);
        linear_pid(4, 0.75, 100, false);
        linear_pid(6, 0.3, 100, false);
        linear_pid(5, 0.3, 80, false);
        pros::delay(300);
    do_turn_global(-90, 0.6, 127);
    linear_pid(30, 0.75, 75, true);
    linear_pid(8, 0.15, 85, true);
    do_turn_global(-90, 0.25, 127);
    linear_pid(15, 0.25, 90, true);
    hold.move(0);
    score.move(92);
    LoaderAir.set_value(false);
    LoaderUp = false;
    pros::delay(1250);
    score.move(120);
    pros::delay(450);
    score.move(120);

    // first 2 loaders done

    linear_pid(12, 0.5, 127, false);
    do_turn_global(182, 0.6, 127);
    linear_pid(118, 2.0, 127, false);
    linear_pid(18, 0.8, 127, true);
    do_turn_global(-90, 0.6, 127);
        score.move(0);
    hold.move(127);
    LoaderAir.set_value(true);
    LoaderUp = true;
    pros::delay(500);
        score.move(0);
    hold.move(127);
    linear_pid(35, 0.95, 35, false);
        linear_pid(0.5, 0.25, 127, true);
        linear_pid(3, 0.25, 127, false);
        linear_pid(1, 0.25, 127, true);
        linear_pid(3, 0.25, 127, false);
        linear_pid(3, 0.25, 127, false);
    do_turn_global(-90, 0.6, 127);
    linear_pid(16, 0.75, 60, true);
    linear_pid(6, 0.5, 60, false);
    LoaderAir.set_value(false);
    LoaderUp = false;

    do_turn_global(180, 0.9, 127); // deg, sec, 127
    hold.move(0);
    linear_pid(14, 0.7, 90, false);
    do_turn_global(98, 1.2, 127); // deg, sec, 127
    linear_pid(90, 2.2, 90, false);
    do_turn_global(180, 0.95, 127); // deg, sec, 127

    linear_pid(8, 0.5, 90, false);
    linear_pid(18.5, 0.8, 90, true);
    do_turn(90, 0.8, 127);
    linear_pid(22, 0.75, 100, true);
    score.move(127);
    pros::delay(750);
        score.move(-127);
        pros::delay(150);
        score.move(127);
        pros::delay(1000);

    LoaderAir.set_value(true);
    LoaderUp = true;
    pros::delay(250);
        score.move(0);
    hold.move(127);
        do_turn_global(90, 0.6, 127);
    linear_pid(38.5, 1.1, 50, false);
        linear_pid(1, 0.25, 120, true);
        linear_pid(3, 0.25, 120, false);
        linear_pid(1, 0.25, 120, true);
        linear_pid(3, 0.125, 127, false);
        linear_pid(1, 0.125, 127, true);
        linear_pid(3, 0.25, 120, false);
        linear_pid(5, 0.25, 127, false);
        pros::delay(200);
    do_turn_global(90.167, 0.7, 127);
    linear_pid(32, 0.8, 80, true);
    do_turn_global(90, 0.5, 127);
    linear_pid(8, 0.25, 90, true);
    hold.move(0);
    score.move(102);
    LoaderAir.set_value(false);
    LoaderUp = false;
    score.move(99);
    linear_pid(8, 0.25, 127, true);
    pros::delay(1470);

    do_turn_global(57, 0.35, 127);
    linear_pid(44, 0.75, 127, false);
    do_turn_global(30, 0.72, 127);
    linear_pid(5, 0.4, 127, false);
    score.move(-127);
    LoaderAir.set_value(true);
    LoaderUp = true;
    pros::delay(200);
    hold.move(-127);
    linear_pid(9, 0.25, 127, true);
    linear_pid(34, 0.75, 127, false);
    linear_pid(8, 0.25, 127, false);
    LoaderAir.set_value(false);
    LoaderUp = false;
}

void elims_right(){ // right side 
    hold.move(127);
    linear_pid(14.67, 0.65, 127, false); // in, sec, 127, backwards
    do_turn_global(30, 0.43, 127); // deg, sec, 127
    LoaderAir.set_value(true);
    LoaderUp = true;
    linear_pid(12.5, 0.5, 127, false); 
    do_turn_global(45, 0.25, 127); 
    linear_pid(16, 0.8, 110, false); 
    score.move(-127);
    pros::delay(300);
    score.move(-65);
    pros::delay(300);
    score.move(0); 
    linear_pid(44, 1.0, 127, true); 
    LoaderAir.set_value(true);
    LoaderUp = true; 
    do_turn_global(-116, 1, 127); 
    linear_pid(7, 0.8, 127, true); 
    do_turn_global(-205, 0.9, 127); 
    hold.move(127);
    linear_pid(15, 0.75, 70, false); 
    // pros::delay(150); //900
    ArmUpAir.set_value(true);
    ArmUp = true; 
    pros::delay(100);
    do_turn_global(-205, 0.8, 127);
    linear_pid(30, 0.9, 127, true);
    score.move_velocity(127); 
    pros::delay(1200);
    score.move_velocity(-127); 
    pros::delay(100);
    hold.move_velocity(127); 
    linear_pid(5, 0.3, 127, false); 
    linear_pid(10, 0.6, 127, true); 
    score.move_velocity(127); 
    pros::delay(1000);
    do_turn_global(-205, 0.7, 127);
    linear_pid(12, 0.65, 127, false);
    do_turn_global(90, 0.75, 127);
    linear_pid(9.9, 0.8, 127, true);
    do_turn_global(180, 0.8, 127);
    linear_pid(28, 3, 75, true);
}

void elims_left(){ // left side
    hold.move(127);
    linear_pid(15.5, 0.67, 127, false); // in, sec, 127, backwards
    do_turn_global(-35, 0.43, 127); // deg, sec, 127
    LoaderAir.set_value(true);
    LoaderUp = true;
    linear_pid(15, 0.5, 90, false); 
    do_turn_global(-45, 0.25, 127); 
    LoaderAir.set_value(false);
    LoaderUp = false;
    do_turn_global(-132, 0.75, 127);
    pros::delay(300);
    LoaderAir.set_value(false);
    LoaderUp = false;
    ArmUpAir.set_value(true);
    ArmUp = true; 
    linear_pid(12.5, 0.8, 80, true);
    score.move(85);
    pros::delay(350);
    score.move(0);
    linear_pid(43, 1.25, 127, false);
    ArmUpAir.set_value(true);
    ArmUp = true;
    do_turn_global(179.5, 1, 127);
    LoaderAir.set_value(true);
    LoaderUp = true;
    pros::delay(350);
    score.move(0);
    hold.move(127);
    linear_pid(34, 1.25, 50, false);

        linear_pid(1, 0.25, 127, true);
        linear_pid(1.5, 0.5, 127, false);
    linear_pid(15, 0.45, 127, true);
    do_turn_global(179, 0.65, 127);
    linear_pid(15, 0.8, 60, true);
    linear_pid(8, 0.3, 60, true);
    score.move(127);
    pros::delay(750);
        score.move(-127);
        pros::delay(150);
        score.move(127);
        pros::delay(900);
}

void soloAWP() {
     hold.move(127);
    linear_pid(12, 0.4, 127, false); // in, sec, 127, backwards
    linear_pid(49, 1.2, 127, true);
    do_turn_global(-91, 0.55, 127); // deg, sec, 127
    LoaderAir.set_value(true);
    LoaderUp = true;
    ArmUpAir.set_value(true);
    ArmUp = true;
    pros::delay(500);
    linear_pid(18, 0.7, 60, false);
    linear_pid(5, 0.25, 75, false);
    pros::delay(400);
    do_turn_global(-90, 0.4, 127);
    do_turn_global(-90, 0.5, 127);
    linear_pid(30, 0.7, 75, true);
    linear_pid(8, 0.2, 60, true);
    hold.move(0);
    score.move(127);
    LoaderAir.set_value(false);
    LoaderUp = false;
    pros::delay(975);

    do_turn_global(4, 1.256, 127); // deg, sec, 127
    score.move(0);
    hold.move(127);
    linear_pid(48, 1.0, 127, false);
    LoaderAir.set_value(true);
    LoaderUp = true;
    ArmUpAir.set_value(false);
    ArmUp = false;
    linear_pid(15, 0.8, 115, false);
    do_turn_global(-45, 0.55, 127); 
    pros::delay(100);
    linear_pid(16, 0.55, 60, true);
    hold.move(0);
    score.move(127);
    pros::delay(250);
    linear_pid(4, 0.2, 80, true);
    ArmUpAir.set_value(true);
    ArmUp = true; 
    score.move(0);
    LoaderAir.set_value(false);
    LoaderUp = false;

    linear_pid(48, 1.35, 127, false);
    do_turn_global(-89, 0.4, 127);
    ArmUpAir.set_value(true);
    ArmUp = true;
    linear_pid(18, 0.5, 80, true);
    score.move(127);
    pros::delay(700);
    //linear_pid(2, 0.8, 127, true); 
}

void trust9ballR() { // right side 
    hold.move(127);
    linear_pid(15.5, 0.65, 127, false); // in, sec, 127, backwards
    do_turn_global(30, 0.43, 127); // deg, sec, 127
    LoaderAir.set_value(true);
    LoaderUp = true;
    linear_pid(12.5, 0.5, 127, false); 
    do_turn_global(45, 0.25, 127); 
    LoaderAir.set_value(false);
    LoaderUp = false;
    linear_pid(21, 0.8, 127, false); 
    do_turn_global(76, 0.55, 127);
    LoaderAir.set_value(true);
    LoaderUp = true;
    // pros::delay(25);
    linear_pid(7.5, 0.8, 127, false); 
    linear_pid(24, 0.8, 127, true);
    do_turn_global(132, 0.75, 127);
    pros::delay(250);
    LoaderAir.set_value(false);
    LoaderUp = false;
    linear_pid(39.5, 1.25, 127, false);
    ArmUpAir.set_value(true);
    ArmUp = true;
    do_turn_global(180, 0.75, 127); 
    linear_pid(16, 0.65, 127, true);
    hold.move(0);
    score.move(127);
    pros::delay(550);
        score.move(-127);
        pros::delay(50);
        score.move(127);
    do_turn_global(180, 0.75, 127);
    LoaderAir.set_value(true);
    LoaderUp = true;
    pros::delay(350);
    score.move(0);
    hold.move(127);
    linear_pid(34, 1.25, 50, false);

        linear_pid(1, 0.25, 127, true);
        linear_pid(1.5, 0.25, 127, false);
    linear_pid(15, 0.45, 127, true);
    do_turn_global(179, 0.65, 127);
    linear_pid(15, 0.8, 60, true);
    score.move(127);
    linear_pid(8, 0.3, 60, true);
}

void trust6wingR() { // right side 
    hold.move(127);
    linear_pid(15.5, 0.65, 127, false); // in, sec, 127, backwards
    do_turn_global(30, 0.43, 127); // deg, sec, 127
    LoaderAir.set_value(true);
    LoaderUp = true;
    linear_pid(12.5, 0.5, 127, false); 
    do_turn_global(46, 0.25, 127); 
    LoaderAir.set_value(false);
    LoaderUp = false;
    linear_pid(20.25, 0.67, 127, false); 
    do_turn_global(76, 0.55, 127);
    linear_pid(6, 0.6, 127, false); 
    pros::delay(50);
    LoaderAir.set_value(true);
    LoaderUp = true;
    linear_pid(2, 0.2, 127, false); 
    linear_pid(25, 0.7, 127, true);
    do_turn_global(132, 0.75, 127);
    pros::delay(250);
    LoaderAir.set_value(false);
    LoaderUp = false;
    linear_pid(39.5, 0.85, 127, false);
    ArmUpAir.set_value(true);
    ArmUp = true;
    do_turn_global(180, 0.75, 127); 
    linear_pid(16, 0.65, 127, true);
    hold.move(0);
    score.move(127);
    pros::delay(500);
        score.move(-127);
        pros::delay(50);
        score.move(127);
        pros::delay(500);
    do_turn_global(180, 0.67, 127);
    
    linear_pid(12, 0.65, 127, false);
    do_turn_global(90, 0.75, 127);
    linear_pid(10, 0.7, 127, true);
    do_turn_global(180, 0.67, 127);
    linear_pid(24, 1.2, 75, true);
}

void trust6wingL() { // left side 
    hold.move(127);
    linear_pid(15.5, 0.65, 127, false); // in, sec, 127, backwards
    do_turn_global(-30, 0.43, 127); // deg, sec, 127
    LoaderAir.set_value(true);
    LoaderUp = true;
    linear_pid(12.5, 0.5, 127, false); 
    do_turn_global(-45, 0.25, 127); 
    LoaderAir.set_value(false);
    LoaderUp = false;
    linear_pid(20, 0.8, 127, false); 
    do_turn_global(-75, 0.55, 127);
    LoaderAir.set_value(true);
    LoaderUp = true;
    // pros::delay(25);
    linear_pid(9, 0.8, 127, false); 
    linear_pid(24, 0.8, 127, true);
    do_turn_global(-132, 0.75, 127);
    pros::delay(250);
    LoaderAir.set_value(false);
    LoaderUp = false;
    linear_pid(39.5, 1.25, 127, false);
    ArmUpAir.set_value(true);
    ArmUp = true;
    do_turn_global(180, 0.75, 127); 
    linear_pid(16, 0.65, 127, true);
    hold.move(0);
    score.move(127);
    pros::delay(550);
        score.move(-127);
        pros::delay(50);
        score.move(127);
    do_turn_global(179.5, 0.75, 127);
    pros::delay(350);
    score.move(0);
    hold.move(127);
    do_turn_global(180, 1.25, 127);
    
    linear_pid(12, 0.5, 127, false);
    do_turn_global(90, 0.5, 127);
    linear_pid(8.5, 0.65, 127, true);
    do_turn_global(178, 0.6, 127);
    linear_pid(28, 3, 65, true);
}

void trust9ballL() { // left side 
    hold.move(127);
    linear_pid(15.5, 0.65, 127, false); // in, sec, 127, backwards
    do_turn_global(-30, 0.43, 127); // deg, sec, 127
    LoaderAir.set_value(true);
    LoaderUp = true;
    linear_pid(12.5, 0.5, 127, false); 
    do_turn_global(-45, 0.25, 127); 
    LoaderAir.set_value(false);
    LoaderUp = false;
    linear_pid(20, 0.8, 127, false); 
    do_turn_global(-75, 0.55, 127);
    LoaderAir.set_value(true);
    LoaderUp = true;
    // pros::delay(25);
    linear_pid(8.5, 0.8, 127, false); 
    linear_pid(24, 0.8, 127, true);
    do_turn_global(-132, 0.75, 127);
    pros::delay(250);
    LoaderAir.set_value(false);
    LoaderUp = false;
    linear_pid(38, 1.25, 127, false);
    ArmUpAir.set_value(true);
    ArmUp = true;
    do_turn_global(180, 0.75, 127); 
    linear_pid(16, 0.65, 127, true);
    hold.move(0);
    score.move(127);
    pros::delay(550);
        score.move(-127);
        pros::delay(50);
        score.move(127);
    do_turn_global(179.5, 1, 127);
    LoaderAir.set_value(true);

    LoaderUp = true;
    pros::delay(350);
    score.move(0);
    hold.move(127);
    linear_pid(34, 1.25, 50, false);

        linear_pid(1, 0.25, 127, true);
        linear_pid(1.5, 0.5, 127, false);
    linear_pid(15, 0.45, 127, true);
    do_turn_global(179, 0.65, 127);
    linear_pid(15, 0.8, 60, true);
    score.move(127);
    linear_pid(8, 0.3, 60, true);
}

void autonomous() {
    // elims_left();

    // soloAWP();
    //trust6wingL();
    // trust9ballL();
    trust6wingR();
    //trust9ballR();
    // elims_right(); 
    // auton_skills();
}

// Driver control sauce
void opcontrol() {

    while (true) {

        //  // Arcade control scheme
        int dir = controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
        int turn = controller.get_analog(ANALOG_LEFT_X);  // Gets the turn left/right from right joystick
        left_mg.move(dir + turn);                      // Sets left motor voltage
        right_mg.move(dir - turn);                     // Sets right motor voltage

        pros::delay(25);

        pros::delay(20);


        if (controller.get_digital(DIGITAL_L1)) { // all running to score
            score.move(127);

        } else if (controller.get_digital(DIGITAL_R1)) { // all running to intake with hood store
            hold.move(127);

        } else if (controller.get_digital(DIGITAL_L2)) { // all running backward to outtake
            score.move(-127);

        } else if (controller.get_digital(DIGITAL_R2)) { // just the intake preroller running to intake
            pre.move(127);

        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { // intake down
            if (ArmDown == false && ArmUp == false) { // from neutral state down
                ArmDownAir.set_value(true);
                pros::delay(20);
                ArmDown = true;
                WingAir.set_value(false);
            } else if (ArmDown == false && ArmUp == true) { // from up position to down
                ArmUpAir.set_value(false);
                pros::delay(20);
                ArmDownAir.set_value(true);
                ArmUp = false;
                ArmDown = true;
                WingAir.set_value(false);
            } else { // from down to neutral
                ArmDownAir.set_value(ArmDown == false);
                pros::delay(20);
                ArmDown = false;
            }
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) { // Intake Up
            if (ArmUp == false && ArmDown == false) { // from neutral to up
                ArmUpAir.set_value(true);
                pros::delay(20);
                ArmUp = true;
            } else if (ArmDown == true && ArmUp == false) { // from down to up
                ArmDownAir.set_value(false);
                pros::delay(20);
                ArmUpAir.set_value(true);
                ArmUp = true;
                ArmDown = false;
            } else { // from up to neutral
                ArmUpAir.set_value(ArmUp == false);
                pros::delay(20);
                ArmUp = false;
            }
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { // toungue mech
            if (LoaderUp == false) {
                LoaderAir.set_value(true);
                pros::delay(20);
                LoaderUp = true;
            } else {
                LoaderAir.set_value(LoaderUp == false);
                pros::delay(20);
                LoaderUp = false;
            }
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { // Wing mech
            if (WingUp == false) {
                WingAir.set_value(true);
                pros::delay(20);
                WingUp = true;
            } else {
                WingAir.set_value(WingUp == false);
                pros::delay(20);
                WingUp = false;
            }
        } else {
            // default stop
            intakePre.move(0);
            intakeMain.move(0);
            intakeHood.move(0);
        }
        }
}
