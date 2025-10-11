#include "drivetrain.h"

// 2. DEFINITION: The members are defined in the constructor

Drivetrain::Drivetrain(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports) 
    : left_motors(left_ports), right_motors(right_ports) {

    }
void Drivetrain::tank_drive(int left_speed, int right_speed) {
    left_motors.move(left_speed);
    right_motors.move(right_speed);
}

void Drivetrain::move_distance(double distance_degrees, int velocity) {
    left_motors.move_relative(distance_degrees, velocity);
    right_motors.move_relative(distance_degrees, velocity);
}

void Drivetrain::brake() {
    left_motors.brake();
    right_motors.brake();
}

double Drivetrain::get_left_position() const {
    auto all = left_motors.get_position_all();
    if (all.empty()) return left_motors.get_position();
    double sum = 0.0;
    for (double v : all) sum += v;
    return sum / static_cast<double>(all.size());
}

double Drivetrain::get_right_position() const {
    auto all = right_motors.get_position_all();
    if (all.empty()) return right_motors.get_position();
    double sum = 0.0;
    for (double v : all) sum += v;
    return sum / static_cast<double>(all.size());
}