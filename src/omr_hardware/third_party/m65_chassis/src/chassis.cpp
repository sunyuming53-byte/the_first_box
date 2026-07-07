// SPDX-License-Identifier: MIT
// m65::Chassis — M65 chassis serial protocol driver (zero ROS deps)
// Extracted from driver_ros.cpp communication logic.
//
// Protocol: 40-byte fixed-length frames over RS232 @ 115200 8N1
//
// SEND frame:    head(2)=0x7F 0x7F, len(1)=0x28, cmd(1)=MSG_ID,
//                data(33 bytes padded), checksum(1), tail(2)=0x0D 0x0A
// RECV response: head(2), len(1), cmd(1), result(1), data(32 bytes),
//                checksum(1), tail(2)

#include "m65/chassis.hpp"

#include <cstdint>
#include <cstring>

#include <array>
#include <iostream>
#include <memory>
#include <mutex>

#include "frame_helpers.hpp"
#include "m65/types.hpp"
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

namespace m65 {

using internal::compute_checksum;
using internal::kChksOffset;
using internal::kChksRangeEnd;
using internal::kCmdOffset;
using internal::kDataOffset;
using internal::kFrameHead0;
using internal::kFrameHead1;
using internal::kFrameLen;
using internal::kFrameTail0;
using internal::kFrameTail1;
using internal::kHeadOffset;
using internal::kLenOffset;
using internal::kResultOffset;
using internal::read_int16_be;
using internal::verify_checksum;
using internal::write_int16_be;

// MSG_IDs from rc_values.h
constexpr uint8_t kMsgIdBaud = 1;
constexpr uint8_t kMsgIdMotorData = 2;
constexpr uint8_t kMsgIdMotorStatus = 3;

// Return codes from rc_values.h
constexpr uint8_t kWSuccess = 0x60;
constexpr uint8_t kWrError = 0x80;

// PWM scale factor from driver_ros.cpp:398-399
constexpr double kPwmScale = 1000.0;

// ── Impl ───────────────────────────────────────────────────────────────

class Chassis::Impl {
public:
    explicit Impl(const ChassisConfig& config) : config_(config), serial_(io_) {}

    ~Impl() { disconnect(); }

    // ── Connection ─────────────────────────────────────────────────

    bool connect() {
        if (serial_.is_open()) {
            disconnect();
        }

        boost::system::error_code ec;
        serial_.open(config_.serial_port, ec);
        if (ec) {
            std::cerr << "[m65::Chassis] open(" << config_.serial_port
                      << ") failed: " << ec.message() << std::endl;
            return false;
        }

        serial_.set_option(boost::asio::serial_port::baud_rate(config_.baud_rate), ec);
        serial_.set_option(boost::asio::serial_port::character_size(8), ec);
        serial_.set_option(
            boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one), ec);
        serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none),
                           ec);
        serial_.set_option(
            boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none),
            ec);

        std::clog << "[m65::Chassis] Serial opened on " << config_.serial_port << " @ "
                  << config_.baud_rate << " baud" << std::endl;

        // Verify baud rate matches hardware
        if (!check_baud()) {
            std::cerr << "[m65::Chassis] Baud check failed on " << config_.serial_port << std::endl;
            disconnect();
            return false;
        }

        serial_id_ = 0;
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_state_ = ChassisState{};
        return true;
    }

    void disconnect() {
        if (serial_.is_open()) {
            boost::system::error_code ec;
            serial_.close(ec);
            if (ec) {
                std::cerr << "[m65::Chassis] close() failed: " << ec.message() << std::endl;
            }
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_state_ = ChassisState{};
        is_connected_ = false;
    }

    bool is_connected() const { return serial_.is_open(); }

    // ── Motion control ────────────────────────────────────────────

    void set_velocity(double linear_x, double angular_z) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_linear_x_ = linear_x;
        cmd_angular_z_ = angular_z;
    }

    // ── State ─────────────────────────────────────────────────────

    ChassisState read_state() {
        if (!serial_.is_open()) {
            std::cerr << "[m65::Chassis] read_state: not connected" << std::endl;
            return ChassisState{};
        }

        // ── Send motor data command with PWM values ──
        double lin_x, ang_z;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            lin_x = cmd_linear_x_;
            ang_z = cmd_angular_z_;
        }

        // Diff-drive kinematics → wheel velocities
        double half_sep = config_.wheel_separation / 2.0;
        double left_vel = (lin_x - ang_z * half_sep) / config_.wheel_radius;
        double right_vel = (lin_x + ang_z * half_sep) / config_.wheel_radius;
        int16_t left_pwm = static_cast<int16_t>(left_vel * kPwmScale);
        int16_t right_pwm = static_cast<int16_t>(right_vel * kPwmScale);

        if (!read_motor_data(left_pwm, right_pwm)) {
            std::cerr << "[m65::Chassis] read_motor_data failed" << std::endl;
            return last_state_;
        }

        // ── Send motor status command ──
        read_motor_status();

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            return last_state_;
        }
    }

    ChassisState last_state() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return last_state_;
    }

    bool is_emergency() const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return last_state_.emergency_status == 1;
    }

private:
    // ── Frame building ────────────────────────────────────────────

    // Builds a full 40-byte frame with command and data payload.
    // cmd: MSG_ID byte placed at byte 3.
    // data: pointer to payload bytes to place starting at byte 4.
    // data_len: number of payload bytes (≤33).
    std::array<uint8_t, kFrameLen> build_frame(uint8_t cmd, const uint8_t* data, size_t data_len) {
        return internal::build_frame(cmd, data, data_len);
    }

    // ── Frame write / read ────────────────────────────────────────

    void write_frame(const std::array<uint8_t, kFrameLen>& frame) {
        boost::system::error_code ec;
        boost::asio::write(serial_, boost::asio::buffer(frame), ec);
        if (ec) {
            std::cerr << "[m65::Chassis] write_frame failed: " << ec.message() << std::endl;
        }
    }

    bool read_frame(std::array<uint8_t, kFrameLen>& frame) {
        boost::system::error_code ec;
        boost::asio::read(serial_, boost::asio::buffer(frame),
                          boost::asio::transfer_exactly(kFrameLen), ec);
        if (ec) {
            std::cerr << "[m65::Chassis] read_frame failed: " << ec.message() << std::endl;
            return false;
        }

        // Validate header
        if (frame[0] != kFrameHead0 || frame[1] != kFrameHead1) {
            std::cerr << "[m65::Chassis] read_frame: bad header (" << std::hex
                      << static_cast<int>(frame[0]) << " " << static_cast<int>(frame[1]) << ")"
                      << std::dec << std::endl;
            return false;
        }

        // Validate checksum
        if (!verify_checksum(frame)) {
            std::cerr << "[m65::Chassis] read_frame: checksum mismatch" << std::endl;
            return false;
        }

        return true;
    }

    // ── Baud check ────────────────────────────────────────────────

    bool check_baud() {
        // Build MSG_ID_GET_BAUD (id=1) frame — no data payload
        auto frame = build_frame(kMsgIdBaud, nullptr, 0);
        write_frame(frame);

        std::array<uint8_t, kFrameLen> response{};
        if (!read_frame(response)) {
            std::cerr << "[m65::Chassis] check_baud: no response" << std::endl;
            return false;
        }

        if (response[kResultOffset] != kWSuccess) {
            std::cerr << "[m65::Chassis] check_baud: response error " << std::hex
                      << static_cast<int>(response[kResultOffset]) << std::dec << std::endl;
            return false;
        }

        // Baud rate in bytes [5:8] (32-bit MSB)
        uint32_t baud = (static_cast<uint32_t>(response[5]) << 24) |
                        (static_cast<uint32_t>(response[6]) << 16) |
                        (static_cast<uint32_t>(response[7]) << 8) |
                        static_cast<uint32_t>(response[8]);

        std::clog << "[m65::Chassis] check_baud: reported baud=" << baud
                  << ", expected=" << config_.baud_rate << std::endl;

        return baud == static_cast<uint32_t>(config_.baud_rate);
    }

    // ── Motor data command ────────────────────────────────────────

    // Sends MSG_ID_GET_MOTOR_DATA with PWM values and parses response.
    // Data payload mapping (bytes 4-36):
    //   [4]        = move_cmd (0)
    //   [5:6]      = move_distance (uint16, 0)
    //   [7]        = mode (0)
    //   [8]        = sonar_switch (0)
    //   [9]        = motor_init (0)
    //   [10:11]    = leftPWM (int16 MSB)
    //   [12:13]    = rightPWM (int16 MSB)
    //   [14:36]    = reserved, zeroed
    bool read_motor_data(int16_t left_pwm, int16_t right_pwm) {
        uint8_t data[33]{};                   // bytes 4-36
        write_int16_be(&data[6], left_pwm);   // offset 10-4=6
        write_int16_be(&data[8], right_pwm);  // offset 12-4=8

        auto frame = build_frame(kMsgIdMotorData, data, sizeof(data));
        write_frame(frame);

        std::array<uint8_t, kFrameLen> response{};
        if (!read_frame(response)) {
            return false;
        }

        if (response[kResultOffset] != kWSuccess) {
            std::cerr << "[m65::Chassis] read_motor_data: command failed " << std::hex
                      << static_cast<int>(response[kResultOffset]) << std::dec << std::endl;
            return false;
        }

        // Verify response cmd matches
        if (response[kCmdOffset] != kMsgIdMotorData) {
            std::cerr << "[m65::Chassis] read_motor_data: wrong cmd "
                      << static_cast<int>(response[kCmdOffset]) << std::endl;
            return false;
        }

        // Parse motor data response (byte offsets from driver_ros.cpp:444-528)
        std::lock_guard<std::mutex> lock(state_mutex_);

        // data bytes are at offsets 5-36 in response
        // (byte 4 = result, already checked)
        last_state_.emergency_status = response[5];
        last_state_.motor_init_status = response[6];
        last_state_.power = response[7];
        last_state_.charge_status = response[8];
        last_state_.left_alarm = response[9];
        last_state_.right_alarm = response[10];
        last_state_.left_encoder = read_int16_be(&response[11]);
        last_state_.right_encoder = read_int16_be(&response[13]);
        last_state_.speed_x = read_int16_be(&response[29]);
        last_state_.speed_th = read_int16_be(&response[31]);
        last_state_.mode = response[33];
        last_state_.version = static_cast<uint16_t>(read_int16_be(&response[35]));

        // Additional fields
        // shutdown @ [28]: 0=ok, 1=shutting, 2=reboot
        // motor_enabled @ [27]: 0=locked, 1=free
        uint8_t shutdown = response[28];
        uint8_t motor_enabled = response[27];
        (void) shutdown;
        (void) motor_enabled;

        return true;
    }

    // ── Motor status command ──────────────────────────────────────

    // Sends MSG_ID_GET_MOTOR_STATUS and parses battery info.
    // Parsing from driver_ros.cpp:612-657.
    bool read_motor_status() {
        uint8_t data[33]{};  // all zeros (no motor_reset, get_ver, etc.)
        auto frame = build_frame(kMsgIdMotorStatus, data, sizeof(data));
        write_frame(frame);

        std::array<uint8_t, kFrameLen> response{};
        if (!read_frame(response)) {
            return false;
        }

        if (response[kResultOffset] != kWSuccess) {
            // Status failures are non-fatal; log and return cached state
            std::clog << "[m65::Chassis] read_motor_status: command failed " << std::hex
                      << static_cast<int>(response[kResultOffset]) << std::dec << std::endl;
            return false;
        }

        // Parse motor status response (byte offsets from driver_ros.cpp:607-656)
        std::lock_guard<std::mutex> lock(state_mutex_);

        // bat_temp @ [5:6] (int16), bat_current @ [7:8] (int16)
        last_state_.battery_temperature = static_cast<double>(read_int16_be(&response[5])) * 0.1;
        last_state_.battery_current = static_cast<double>(read_int16_be(&response[7])) * 0.01;

        return true;
    }

    // ── Members ───────────────────────────────────────────────────

    ChassisConfig config_;

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;

    // Command state protected by cmd_mutex_
    mutable std::mutex cmd_mutex_;
    double cmd_linear_x_ = 0.0;
    double cmd_angular_z_ = 0.0;

    // Cached state protected by state_mutex_
    mutable std::mutex state_mutex_;
    ChassisState last_state_;

    uint8_t serial_id_ = 0;
    bool is_connected_ = false;
};

// ── Chassis facade ─────────────────────────────────────────────────────

Chassis::Chassis(const ChassisConfig& config) : impl_(std::make_unique<Impl>(config)) {}

Chassis::~Chassis() = default;

bool Chassis::connect() { return impl_->connect(); }
void Chassis::disconnect() { impl_->disconnect(); }
bool Chassis::is_connected() const { return impl_->is_connected(); }

void Chassis::set_velocity(double linear_x, double angular_z) {
    impl_->set_velocity(linear_x, angular_z);
}

ChassisState Chassis::read_state() { return impl_->read_state(); }
ChassisState Chassis::last_state() const { return impl_->last_state(); }
bool Chassis::is_emergency() const { return impl_->is_emergency(); }

}  // namespace m65
