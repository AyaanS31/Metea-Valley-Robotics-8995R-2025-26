#include "main.h"
#include <cmath>

// controller 676767
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups and sensors 
pros::MotorGroup left_mg({-11, -12, -13}, pros::MotorGear::blue);
pros::MotorGroup right_mg({15, 16, 17}, pros::MotorGearset::blue); 

pros::Imu imu_sensor(21);
pros::Distance left_dist_sensor(8);
pros::Distance right_dist_sensor(20);

pros::Motor intake(10);
pros::Motor lever(-14);

// Variable initialization for the macros
bool ArmUp = true;
bool LoaderUp = false;
bool WingUp = false;
bool LeverUp = false;
bool LeverMovingDown = false;
int leverStopCount = 0;
bool MidGoal = false;

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 14, not reversed
pros::Rotation horizontalEnc(19); // horiztonal 
pros::Rotation verticalEnc(20); // vertical

pros::ADIDigitalOut LeverAir('H', false);
pros::ADIDigitalOut WingAir('A', false);
pros::ADIDigitalOut LoaderAir('G', false);
pros::ADIDigitalOut HoodAir('F', true);

// forwrad/backward PID tuning values
double kP_linear = 22;
double kI_linear = 0.001; // already almost no overshoot
double kD_linear = 2;

// turning PID tuning values
double kP_turn = 5;
double kI_turn = 0.125;
double kD_turn = .32;

// distance sensor offsets
const double MIDDLE_Y_OFFSET = 7.1; 
const double LEFT_Y_OFFSET = 7.1;   
const double LEFT_X_OFFSET = 5.2;   

const float wheel_diameter = 3.25*0.99576271185; // in inches
const float wheel_circumference = wheel_diameter * M_PI; // in inches
const float degreesPerInch = 360.0 / wheel_circumference; // degrees per inch of travel
const float motor_degree_to_inch = 1.0 / degreesPerInch; // inches per degree of motor rotation
const float inches_per_tick = (wheel_diameter * M_PI )/ 36000.0; // inches per encoder tick in motor encoders
const double linear_error_threshold = .3; // inches
const double linear_settle_time = 3; // seconds
const double linear_integral_max = 50.0; // max integral term to prevent windup

const double angular_error_threshold = 1.0 * M_PI / 180.0; // radians (1 degree)
const double ANGULAR_SETTLE_TIME = 0.3; // seconds
const double angular_integral_max = 0.5; // max integral term to prevent wind

float intake_speed; 
// Global position variables
std::vector<double> global_position = {0.0, 0.0}; // (x, y) in inches
float global_heading = 0; // In radians, facing east
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
    const float wheel_distance = 10.0; // Distance between left and right wheels in inches
    const float wheel_circumference = 3.25 * M_PI; // Assume 3.25 inch wheels
    const float ticks_per_revolution = 360.0; // Encoder ticks per wheel revolution
    float prev_left = 0.0, prev_right = 0.0;

    while (true) {
        float current_left = (verticalEnc.get_position()/100.0) * motor_degree_to_inch; // vertical tracking wheel for forward/backward
        float current_right = (horizontalEnc.get_position()/100.0) * motor_degree_to_inch; // horizontal tracking wheel for strafing, but also gives better forward/backward readings than vertical

        float delta_left = (current_left - prev_left);
        float delta_right = (current_right - prev_right);

        prev_left = current_left;
        prev_right = current_right;

        float delta_theta = (delta_right - delta_left) / wheel_distance;
        global_heading += delta_theta;
        global_heading = wrap_angle(global_heading);

        float avg_position = (delta_left + delta_right) / 2.0;
        float dx = avg_position * cos(global_heading);
        float dy = avg_position * sin(global_heading);
        global_position[0] += dx;
        global_position[1] += dy;
        pros::delay(10);
    }
}

void initialize() {
    pros::lcd::initialize();
    imu_sensor.reset();
    pros::Task odom_task(odometry_task);
}

// main move function
void linear_pid(float target_distance, float drive_timeout, double speed, bool backwards = false) { //linear PID

    if (backwards) target_distance = -target_distance;
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

// main turn function
void do_turn_global(double target_global_heading_deg, double turn_timeout, double max_speed) {
    // Convert target to radians if your global_heading is in radians
    // Otherwise keep degrees

    left_mg.tare_position();
    right_mg.tare_position();
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

// distance to the walls
double get_corrected_dist() {
    double center_raw = right_dist_sensor.get() / 25.4; // ts converts to inches 
    double left_raw = left_dist_sensor.get() / 25.4; 

    bool center_valid = (center_raw > 1.0 && center_raw < 85.0);
    bool left_valid = (left_raw > 1.0 && left_raw < 85.0);

    // offset nonesense 
    if (center_valid && left_valid) {
        double dist1 = center_raw + MIDDLE_Y_OFFSET;
        double dist2 = left_raw + LEFT_Y_OFFSET;
        return (dist1 + dist2) / 2.0;
    } else if (center_valid) {
        return center_raw + MIDDLE_Y_OFFSET;
    } else if (left_valid) {
        return left_raw + LEFT_Y_OFFSET;
    }
    return -1; 
}

// Linear P loop based on distance sensors
void dist_to_back(double target_dist_from_wall, double timeout) {
    double current_dist = get_corrected_dist();
    if (current_dist == -1) return; 

    double travel_needed = target_dist_from_wall - current_dist;

    if(std::abs(travel_needed) < 0.9) return;

    linear_pid(travel_needed, timeout, 35, false);

    left_mg.tare_position();
    right_mg.tare_position();
}

// Angular P loop based on distance sensors
void square_to_wall(double timeout) {
    uint32_t start_time = pros::millis();
    double kP_tilt = 15; 

    while (pros::millis() - start_time < timeout) {
        double L_raw = left_dist_sensor.get() / 25.4;
        double C_raw = right_dist_sensor.get() / 25.4;

        if (L_raw > 70 || C_raw > 70) break; 
        // error with offset 
        double error = (L_raw - C_raw);
        
        //if (std::abs(error) < 0.05) break; 

        double output = error * kP_tilt;
        
        if(std::abs(output) < 8) output = std::copysign(8, output);

        left_mg.move(output);
        right_mg.move(-output);

        pros::delay(20);
    }
    left_mg.brake();
    right_mg.brake();
}

// autonomous routines
void auton_skills() {
    // square_to_wall(200);
    // dist_to_back(36, 0.25);
    // do_turn_global(-88.5, 0.65, 80);

    HoodAir.set_value(false);
    linear_pid(14, 0.5, 80, false);
    do_turn_global(-45, 0.5, 80);
    intake.move(127);
    linear_pid(6, 0.35, 127, false);
    linear_pid(6, 0.45, 127, false);
        intake.move(0);
    linear_pid(4, 0.35, 127, false);
    LoaderAir.set_value(true);
    LoaderUp = true;
    do_turn_global(-135, 0.65, 80);
    intake.move(127);
    linear_pid(14.5, 0.65, 127, true);
    lever.move(127);
        pros::delay(100);
        lever.move(-127);
        pros::delay(50);
        lever.move(127);
        pros::delay(200);
        lever.move(0);
        pros::delay(20);
        HoodAir.set_value(true);
        lever.move(50);
        pros::delay(500);
    lever.move(-127);

    LeverAir.set_value(true);
    LeverUp = true;
    WingAir.set_value(true);
    WingUp = true;
    intake.move(0);
    linear_pid(51.25, 1.5, 127, false);
    HoodAir.set_value(false);
    intake.move(127);
    do_turn_global(-180, 0.65, 100);

    linear_pid(20, 0.65, 50, false);
        linear_pid(3, 0.25, 80, false);
        linear_pid(1, 0.25, 50, true);
        linear_pid(3, 0.5, 100, false);
        linear_pid(1, 0.25, 50, true);
        linear_pid(3, 0.5, 100, false);
    do_turn_global(120, 0.65, 127);
    linear_pid(14, 0.8, 80, true);
    LoaderAir.set_value(false);
    LoaderUp = false;
    do_turn_global(177, 0.5, 100);

    linear_pid(75, 2.5, 127, true);
    do_turn_global(90, 0.75, 100);
    linear_pid(14, 0.65, 90, false);
    dist_to_back(14, 0.5); // idk 

    do_turn_global(0, 0.65, 127);
    linear_pid(24, 1, 80, true);
    HoodAir.set_value(true);
    intake.move(0);
    lever.move(127);
    pros::delay(100);
    lever.move(50);
    linear_pid(5, 0.2, 100, true);
    pros::delay(500);
    lever.move(-127);

    // move to second loader and back

    // fried ahh clear park zone

    // score middle goal
    
    // third loader

    // go and score

    // fourth loader and score

    // clear and park
}

void solo_awp(){

    intake.move(127);
    //square_to_wall(200);
    linear_pid(11, 0.33, 65, false); // in, sec, 127, backwards
    dist_to_back(36, 0.25);
    //square_to_wall(250);
    //do_turn_global(0.5, 0.43, 100); // deg, sec, 127

    linear_pid(34, 0.7, 127, true);
    //square_to_wall(500);
    dist_to_back(28.5, 0.5);
    LeverAir.set_value(true);
    LeverUp = true;
    WingAir.set_value(true);
    WingUp = true;
    do_turn_global(-88.5, 0.65, 80); // deg, sec, 127
    LoaderAir.set_value(true);
    LoaderUp = true;
    pros::delay(150);
    linear_pid(19, 0.55, 50, false);
        linear_pid(3, 0.25, 80, false);
        do_turn_global(-88, 0.25, 127);
        linear_pid(1, 0.05, 127, true); 
        linear_pid(3, 0.15, 127, false); 
        do_turn_global(-88.75, 0.1, 127);
    linear_pid(39, 0.5, 100, true);
    HoodAir.set_value(true);
    intake.move(0);
    lever.move(80);
    linear_pid(5, 0.2, 100, true);
    pros::delay(250);
    LoaderAir.set_value(false);
    LoaderUp = false;

    linear_pid(12, 0.4, 127, false); 
    lever.move(-127);
    do_turn_global(26.5, 0.7, 95);
    intake.move(127);
    HoodAir.set_value(false);
    linear_pid(28, 0.5, 127, false);
    LoaderAir.set_value(true);
    LoaderUp = true;
    linear_pid(8, 0.3, 127, false); 

    // loader done 
    // go to middle goal
    LeverAir.set_value(false);
    LeverUp = false;
    LoaderAir.set_value(false);
    LoaderUp = false; 
    do_turn_global(-1.25, 0.4, 127); 
    linear_pid(40, .8, 127, false);
    LoaderAir.set_value(true);
    LoaderUp = true;
    linear_pid(15.5, 0.5, 127, false); 
    do_turn_global(-45, 0.45, 127); 
    linear_pid(13.5, 0.45, 60, true);
    HoodAir.set_value(true);
    lever.move(127);
    pros::delay(150);
    intake.move(0);
    pros::delay(100);
    lever.move(80);
    pros::delay(250); 
    HoodAir.set_value(false);
    LoaderAir.set_value(false);
    LoaderUp = false;
    lever.move(-127);
    LeverAir.set_value(true);
    LeverUp = true;
    intake.move(127);
    // going to the loader for long goal 

    linear_pid(44.75, 1, 127, false);
    LoaderAir.set_value(true);
    LoaderUp = true;
    do_turn_global(-90, 0.35, 127);
    linear_pid(12, 0.3, 127, false);
    linear_pid(12, 0.125, 80, false);
        linear_pid(3, 0.1, 100, false);
        lever.move(50);
        pros::delay(25);
        lever.move(-50); 
        linear_pid(1, 0.05, 127, true);
        lever.move(0);
        linear_pid(3, 0.05, 127, false);
        do_turn_global(-95, 0.25, 127);
    linear_pid(60, 0.5, 127, true);
    LoaderAir.set_value(false);
    LoaderUp = false;
    HoodAir.set_value(true);
    intake.move(0);
    lever.move(127);
    linear_pid(5, 0.2, 100, true);
}

void elite_elims_right(){
    intake.move(127);
    linear_pid(30, 0.65, 100, false); // in, sec, 127, backwards
    dist_to_back(40, 0.4);
    square_to_wall(0.5);
    do_turn_global(20, 0.43, 100); // deg, sec, 127
    LoaderAir.set_value(true);
    LoaderUp = true;
    linear_pid(12.5, 0.5, 100, false); 
    do_turn_global(45, 0.25, 100); 
    LoaderAir.set_value(false);
    LoaderUp = false;
    linear_pid(21, 0.8, 100, false); 
    do_turn_global(56, 0.55, 100);
    LoaderAir.set_value(true);
    LoaderUp = true;
    // pros::delay(25);
    linear_pid(7.5, 0.8, 100, false); 
    linear_pid(24, 0.8, 100, true);
    do_turn_global(132, 0.75, 100);
    // pros::delay(250);
    // LoaderAir.set_value(false);
    // LoaderUp = false;
    // linear_pid(39.5, 1.25, 127, false);
    // ArmUpAir.set_value(true);
    // ArmUp = true;
    // do_turn_global(180, 0.75, 127); 
    // linear_pid(16, 0.65, 127, true);
    // intake.move(0);
    // score.move(127);
    // pros::delay(550);
    //     score.move(-127);
    //     pros::delay(50);
    //     score.move(127);
    // do_turn_global(180, 0.75, 127);
    // LoaderAir.set_value(true);
    // LoaderUp = true;
    // pros::delay(350);
    // score.move(0);
    // intake.move(127);
    // linear_pid(34, 1.25, 50, false);

    //     linear_pid(1, 0.25, 127, true);
    //     linear_pid(1.5, 0.25, 127, false);
    // linear_pid(15, 0.45, 127, true);
    // do_turn_global(179, 0.65, 127);
    // linear_pid(15, 0.8, 60, true);
    // score.move(127);
    // linear_pid(8, 0.3, 60, true);
}

void elite_elims_left(){

}

void autonomous() {
    //solo_awp();
    auton_skills();
}

// driver control macros and movement
void opcontrol() {
    HoodAir.set_value(false); 
    while (true) {

        // single stick arcade drive
        int dir = controller.get_analog(ANALOG_LEFT_Y);
        int turn = controller.get_analog(ANALOG_LEFT_X);
        left_mg.move(dir + turn);
        right_mg.move(dir - turn);

        

        if (controller.get_digital(DIGITAL_L1) && !LeverUp) {
            intake.move(127);
        }

        else if (controller.get_digital(DIGITAL_L2)) {
            intake.move(-75);
        }

        else if (controller.get_digital_new_press(DIGITAL_R1)) { // regular scoring macro
            intake.move(127);
            pros::delay(80);    
            HoodAir.set_value(true);

            LeverUp = true;
            leverStopCount = 0;
            lever.move(68);

            intake.move(0);
        }

        else if (controller.get_digital_new_press(DIGITAL_R2)) { // lever arm change height macro
            if (ArmUp) {
                LeverAir.set_value(false);
                pros::delay(20);
                ArmUp = false;
                WingAir.set_value(false); //going undergoal
            } else {
                LeverAir.set_value(true);
                pros::delay(20);
                ArmUp = true;
                WingAir.set_value(true); // deafult for the wing is the up position
            }
        }

        else if (controller.get_digital(DIGITAL_X)){ // fast scoring macro / ping pong tech
            intake.move(127);
            pros::delay(80);    
            HoodAir.set_value(true);

            LeverUp = true;
            leverStopCount = 0;
            lever.move(127);

            intake.move(0);
        }

         else if (controller.get_digital(DIGITAL_Y)){ // really slow scoring for middle goal 7
            MidGoal = true;
            intake.move(127);
            pros::delay(80);    
            HoodAir.set_value(true);

            LeverUp = true;
            leverStopCount = 0;
            lever.move(127); // overcome any jammed Blocks/ overheated lever motor
            pros::delay(100); 
            lever.move(35); // main scoring speed at ~25% 

            intake.move(0);
        }

         else if (LeverUp && MidGoal) { //when does it give up if it is trying to score middle 7
            if (abs(lever.get_actual_velocity()) < 5) {
                leverStopCount++;
            } else {
                leverStopCount = 0;
            }

            if (leverStopCount >= 5) { // 2.5 seconds count 
                LeverUp = false;
                LeverMovingDown = true;
                leverStopCount = 0;

                lever.move(-127);
            }
        }

        else if (controller.get_digital(DIGITAL_A)) { // default up, toggle to down
                WingAir.set_value(false);
                pros::delay(20);
        }

        else if (controller.get_digital_new_press(DIGITAL_B)) { // Loader Mech down/up
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

        else if (LeverUp) {   // if lever trying to move up, when does it return down
            if (abs(lever.get_actual_velocity()) < 10) { // actual velocity < expected velocity: jam or hardstop
                leverStopCount++;
            } else {
                leverStopCount = 0;
            }

            if (leverStopCount >= 5) {
                LeverUp = false;
                LeverMovingDown = true;
                leverStopCount = 0;

                HoodAir.set_value(false);
                lever.move(-127);
            }
        }

        else if (LeverMovingDown) { // trying to move down, when does it hit hardstop
            if (abs(lever.get_actual_velocity()) < 10) {
                leverStopCount++;
            } else {
                leverStopCount = 0;
            }

            if (leverStopCount >= 3) {
                LeverMovingDown = false;
                leverStopCount = 0;
                lever.move(-5); // small force down in case it bounces up 
            }
        }

        else if (controller.get_digital(DIGITAL_DOWN)) { // open the hood for taking Blocks in goal
                 HoodAir.set_value(true);
                 pros::delay(20);
         }


        else {
            intake.move(0); // stop intake when we dont need it
		    HoodAir.set_value(false); // close hood for default state

            if(ArmUp) { // only if we arent trying to go undergoal
                WingAir.set_value(true); // deafult for the wing is the up position
            }
        }

        pros::delay(25);
    }
}