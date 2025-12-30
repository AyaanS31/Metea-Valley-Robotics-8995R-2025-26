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
pros::Rotation horizontalEnc(14);

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

 // Utility functions
double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

double wrapRad(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
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

void odometry_task();
void initialize() {
    pros::lcd::initialize();
    imu_sensor.reset();
    int leftSevenWingAngle = 115;
    double leftSevenWingX = -47.89;
    double leftSevenWingY = -10.15;

    int soloAWP = 0;

    imu_sensor.set_heading(leftSevenWingAngle);
    global_position[0] = leftSevenWingX;
    global_position[1] = leftSevenWingY;



    pros::Task lcd_task([] {
    while (true) {
        pros::lcd::print(0, "X: %.1f", left_mg.get_position()*24/941);
        pros::lcd::print(1, "Y: %.1f", right_mg.get_position()*24/941);
        pros::delay(20);
    }
});



}

void odometry_task() {
    double lastForward = 0;
    double lastStrafe = 0;

    left_mg.tare_position();
    right_mg.tare_position();
    horizontalEnc.reset_position();

    while (true) {
        // Heading (radians, signed)
        double headingDeg = imu_sensor.get_heading();
        global_heading = wrapRad(degToRad(headingDeg));


        // Forward distance (inches)
        double forward = ((left_mg.get_position() + right_mg.get_position()) / 2.0)
                          * motor_degree_to_inch;

        // Strafe distance (inches)
        double strafe = (horizontalEnc.get_position() / 100.0)
                         * motor_degree_to_inch;

        double dF = forward - lastForward;
        double dS = strafe - lastStrafe;

        lastForward = forward;
        lastStrafe = strafe;

        // Rotate into field frame
        double dx = dF * cos(global_heading) - dS * sin(global_heading);
        double dy = dF * sin(global_heading) + dS * cos(global_heading);

        global_position[0] += dx;
        global_position[1] += dy;

        pros::delay(10);
    }
}

void disabled() {}

void competition_initialize() {}


// PID CODE
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

void do_turn(double target_angle, double turn_timeout, double max_speed) {
    double start_heading = wrapRad(degToRad(imu_sensor.get_rotation()));
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

        double current_heading = wrapRad(degToRad(imu_sensor.get_heading()));
        turn_error = wrapRad(target_angle - current_heading);


        // Deadzone to prevent vibration
        if (std::abs(turn_error) < degToRad(1.0)) turn_error = 0.0;

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
    double target_heading = wrapRad(degToRad(target_global_heading_deg));
    double current_heading = wrapRad(degToRad(imu_sensor.get_heading()));

    double turn_error = wrapRad(target_heading - current_heading);
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

        current_heading = wrapRad(degToRad(imu_sensor.get_rotation()));
        turn_error = wrapRad(target_heading - current_heading); // shortest-path error

        // Deadzone to prevent vibration
        if (std::abs(turn_error) < degToRad(1.0)) turn_error = 0.0;


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

        if (std::abs(turn_error) < degToRad(1.0)) settle_timer += dt;
        else settle_timer = 0.0;

        if (settle_timer >= ANGULAR_SETTLE_TIME) is_settled = true;

        pros::lcd::print(0, "Heading: %f | Error: %f", current_heading, turn_error);
        pros::delay(20);
    }

    left_mg.brake();
    right_mg.brake();
}

void drive_to_point(double targetX, double targetY, double speed = 127, double timeout = 5.0) {
    // Calculate distance from current position to target
    double dx = targetX - global_position[0];
    double dy = targetY - global_position[1];
    double distance = sqrt(dx * dx + dy * dy); // straight-line distance in inches

    // Determine if driving backward (optional: you can implement heading-based check)
    bool backward = false;

    // Run your existing linear PID with the computed distance and timeout
    linear_pid(distance, timeout, speed, backward);
}

// cordinates are based on path route from pathjerry.io 

void auton_skills() {}

void elims(){
    hold.move(127);
    linear_pid(24, 1.0, 127, false);
    LoaderAir.set_value(true);
    LoaderUp = true; 
    pros::delay(500);
    do_turn_global(45, 0.9, 127);
    LoaderAir.set_value(false);
    LoaderUp = false; 
    linear_pid(20, 0.6, 127, false);
    hold.move(-127);
    pros::delay(300);
    linear_pid(48, 0.9, 127, true);
    do_turn_global(-90, 0.8, 127);
    LoaderAir.set_value(true);
    LoaderUp = true; 
    hold.move(127); 
    linear_pid(8, 0.8, 127, false);
    pros::delay(500);
    ArmUpAir.set_value(true);
    ArmUp = true; 
    linear_pid(22, 0.9, 127, true);
    score.move(127);
}

void left_seven_wing(){
    drive_to_point(-33.7,-17.3, 127,1); 
    LoaderAir.set_value(true);
    LoaderUp = true; 
}

void autonomous() {
    left_seven_wing();
    // hold.move(127);
    // drive_to_point(48,0, 127,5); 
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

        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) { // intake down
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
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) { // Intake Up
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
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { // Wing mech
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
