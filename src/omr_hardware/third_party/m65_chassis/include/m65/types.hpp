#pragma once

#include <cstdint>

#include <string>

namespace m65 {

// Configuration for the M65 chassis serial interface.
// Follows the same pattern as dais::MotorConfig (zero ROS deps).
struct ChassisConfig {
    std::string serial_port = "/dev/ttyBase";
    int baud_rate = 115200;
    double wheel_separation = 0.355;  // m, from driver_ros.cpp:89 (wheel_track_)
                                      // TODO: measure actual
    double wheel_radius = 0.0625;     // m, from driver_ros.cpp:86 (wheel_diameter_/2)
                                      // TODO: measure actual
    int encoder_cpr = 0;              // pulses per wheel revolution — must be set
                                      // to actual hardware value before use
};

// Parsed motor data from the 40-byte response packet (MSG_ID_GET_MOTOR_DATA).
// Byte offsets reference driver_ros.cpp:444-528.
//
// Additional fields from motor status packet (MSG_ID_GET_MOTOR_STATUS)
// reference driver_ros.cpp:607-656.
struct ChassisState {
    // --- Motor data packet (bytes 5-36) ---
    uint8_t emergency_status = 0;   // byte 5
    uint8_t motor_init_status = 0;  // byte 6
    uint8_t power = 0;              // byte 7
    uint8_t charge_status = 0;      // byte 8
    uint8_t left_alarm = 0;         // byte 9
    uint8_t right_alarm = 0;        // byte 10
    int16_t left_encoder = 0;       // bytes 11-12
    int16_t right_encoder = 0;      // bytes 13-14
    int16_t speed_x = 0;            // bytes 29-30, PWM units
    int16_t speed_th = 0;           // bytes 31-32, PWM units
    uint8_t mode = 0;               // byte 33
    uint16_t version = 0;           // bytes 35-36

    // --- Motor status packet (additional fields) ---
    double battery_temperature = 0.0;  // motor_status bytes 5-6, °C
    double battery_current = 0.0;      // motor_status bytes 7-8, A

    // --- Reserved / padding ---
    uint8_t sonar_switch = 0;  // reserved for V2, init to 0
    uint8_t reserved[3] = {};  // padding to maintain alignment
};

}  // namespace m65
