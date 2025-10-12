#include "main.h"
#include <cmath>
#include <vector>
// Assume this includes the Drivetrain class definition
#include "drivetrain.h" 
#include "pros/adi.h"
#include "pros/motors.h"

// --- GLOBAL STATE & SENSORS ---

// Define the global drivetrain pointer (must be initialized in initialize())
Drivetrain* drivetrain = nullptr;

// 1. Pose struct (Represents the robot's global position)
struct Pose { 
    double x = 0.0;     // X position (inches)
    double y = 0.0;     // Y position (inches)
    double theta = 0.0; // Orientation (radians)
};
Pose current_pos;

// 2. Sensor Definitions (ADI Encoders for Translation)
// NOTE: Change these placeholder ports to match your hardware setup!
// Vertical Pod (Forward/Backward movement)
pros::ADIEncoder encoder_vertical('A', 'B', false);
// Horizontal Pod (Strafe/Sideways movement)
pros::ADIEncoder encoder_strafe('C', 'D', false);
// Inertial Sensor for heading correction (optional, but good practice)
pros::Imu imu_sensor(7); 
// Other sensors (like distance)
pros::Distance distance_sensor(8);

// 3. Calibration Constants (CRITICAL FOR ACCURACY)
const double TRACK_WIDTH = 12.5;         // Distance between left/right motor centers (in)
const double STRAFE_CENTER_OFFSET = 3.0; // Distance from robot center to horizontal pod axle (in)

// Conversion factors: (Wheel Diameter * PI) / Ticks Per Rev
// Assuming ADI Encoders (360 ticks) and 2.75" wheels
const double POD_TICK_TO_INCH = (2.75 * M_PI) / 360.0; 
// Assuming V5 Motors set to report in DEGREES (360 degrees per rev) and 4" wheels
const double MOTOR_TICK_TO_INCH = (4.0 * M_PI) / 360.0; 

// Internal variables to hold sensor readings from the previous loop iteration
double last_vertical_ticks = 0.0;
double last_strafe_ticks = 0.0;
double last_left_motor_pos = 0.0; 
double last_right_motor_pos = 0.0; 
/*
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

// -- DISTANCE SENSOR CORRECTION FUNCTION -- 



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
	imu_sensor.reset(); // Reset IMU at start 

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


void autonomous() {}

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

        // Display odometry position on the controller (optional)
        master.set_text(0, 0, "X:" + std::to_string(int(current_pos.x)) + " Y:" + std::to_string(int(current_pos.y)));

        pros::delay(20);
    }
}