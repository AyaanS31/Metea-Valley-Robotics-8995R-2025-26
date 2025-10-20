#include "drivetrain.h"
#include "exception.h"
#include <stdexcept>

// 2. DEFINITION: The members are defined in the constructor

Drivetrain::Drivetrain(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports)
    : left_motors(left_ports), right_motors(right_ports) {
    // Basic validation: ensure both sides have at least one motor port
    if (left_ports.empty() || right_ports.empty()) {
        throw HardwareError("Drivetrain constructed with empty motor port list");
    }
}

void Drivetrain::tank_drive(int left_speed, int right_speed) {
    try {
        left_motors.move(left_speed);
        right_motors.move(right_speed);
    } catch (const std::exception& e) {
        throw HardwareError(std::string("tank_drive failed: ") + e.what());
    }
}

void Drivetrain::move_distance(double distance_degrees, int velocity) {
    try {
        left_motors.move_relative(distance_degrees, velocity);
        right_motors.move_relative(distance_degrees, velocity);
    } catch (const std::exception& e) {
        throw HardwareError(std::string("move_distance failed: ") + e.what());
    }
}

void Drivetrain::brake() {
    try {
        left_motors.brake();
        right_motors.brake();
    } catch (const std::exception& e) {
        throw HardwareError(std::string("brake failed: ") + e.what());
    }
}

double Drivetrain::get_left_position() const {
    try {
        auto all = left_motors.get_position_all();
        if (all.empty()) return left_motors.get_position();
        double sum = 0.0;
        for (double v : all) sum += v;
        return sum / static_cast<double>(all.size());
    } catch (const std::exception& e) {
        throw HardwareError(std::string("get_left_position failed: ") + e.what());
    }
}

double Drivetrain::get_right_position() const {
    try {
        auto all = right_motors.get_position_all();
        if (all.empty()) return right_motors.get_position();
        double sum = 0.0;
        for (double v : all) sum += v;
        return sum / static_cast<double>(all.size());
    } catch (const std::exception& e) {
        throw HardwareError(std::string("get_right_position failed: ") + e.what());
    }
}