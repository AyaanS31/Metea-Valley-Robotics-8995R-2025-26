#pragma once
#include <stdexcept>
#include <string>

// Small set of custom exception types for clearer error handling in robot code
struct SensorError : public std::runtime_error {
    explicit SensorError(const std::string& msg) : std::runtime_error("SensorError: " + msg) {}
};

struct HardwareError : public std::runtime_error {
    explicit HardwareError(const std::string& msg) : std::runtime_error("HardwareError: " + msg) {}
};

struct LogicError : public std::runtime_error {
    explicit LogicError(const std::string& msg) : std::runtime_error("LogicError: " + msg) {}
};
