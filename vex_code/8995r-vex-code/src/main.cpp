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

// PID constants
double kP_linear = 10;
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
        global_heading = wrap_angle( (imu_sensor.get_rotation()) / 2 * M_PI / 180.0);
        
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
    imu_sensor.reset();
    pros::delay(1000); // wait for imu to calibrate

    imu_sensor.set_heading(70); // set initial heading to 70 degrees

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

// auton skills code 

// void autonomous_skills(){
//     chassis.setPose(0, 0, 0);
//     chassis.moveToPoint(0, 15, 700);
//     chassis.turnToHeading(90, 1500);
//     chassis.moveToPoint(-12, 15, 700, {.maxSpeed = 50}); 
// }


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
    imu_sensor.set_heading(0); // set initial heading to 0 degrees

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
    global_position[0] = 0; 
    global_position[1] = 0;    
    global_heading = 0;

    drive_to(12, 0, 2000, 1.0); 

    // // move to field balls 
    // drive_to(-26.411, 21.054, 3000, 1.0); 
    // turn_to((16*M_PI)/9, 2000);

    // // move to loader 
    // drive_to(-48.632, 47.11, 3000, 1.0);
    // turn_to((3*M_PI)/2, 2000);
    // drive_to(-58.155, 46.714, 3000, 1.0);
    // // intake from loaders 

    // drive_to(-28.196, 47.64, 3000, -1.0);
    // // outtake to long goal 
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
