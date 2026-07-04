#include "realman/node/arm_node.hpp"
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>

namespace rm {
namespace {

struct CalibrationData {
    std::array<double, 9> rot_matrix{1,0,0, 0,1,0, 0,0,1};
    std::array<double, 3> translation{0, 0, 0};
};

std::string parseArrayData(const std::string& txt, const std::string& key) {
    auto pos = txt.find(key);
    if (pos == std::string::npos) return "";
    auto start = txt.find('[', pos);
    auto end = txt.find(']', start);
    if (start == std::string::npos || end == std::string::npos) return "";
    return txt.substr(start + 1, end - start - 1);
}

void parseDoubles(const std::string& data, double* out, size_t count) {
    std::istringstream ss(data);
    std::string token;
    size_t i = 0;
    while (std::getline(ss, token, ',') && i < count) {
        size_t s = token.find_first_not_of(" \t\n\r");
        size_t e = token.find_last_not_of(" \t\n\r");
        if (s != std::string::npos && e != std::string::npos) {
            out[i] = std::stod(token.substr(s, e - s + 1));
        }
        i++;
    }
}

CalibrationData loadCalibrationYAML(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open calibration file: " + path);
    }

    CalibrationData calib;
    std::string line;
    bool in_rotation = false, in_translation = false;
    std::string rot_data, trans_data;

    while (std::getline(file, line)) {
        if (line.find("rotation_matrix:") != std::string::npos) {
            in_rotation = true;
            in_translation = false;
        } else if (line.find("translation_vector:") != std::string::npos) {
            in_translation = true;
            in_rotation = false;
        } else if (in_rotation || in_translation) {
            auto& target = in_rotation ? rot_data : trans_data;
            target += line + "\n";
        }
    }

    std::string rdata = parseArrayData(rot_data, "data:");
    if (!rdata.empty()) {
        parseDoubles(rdata, calib.rot_matrix.data(), 9);
    }

    std::string tdata = parseArrayData(trans_data, "data:");
    if (!tdata.empty()) {
        parseDoubles(tdata, calib.translation.data(), 3);
    }

    return calib;
}

// Rotation matrix (row-major 3x3) to quaternion (x, y, z, w)
std::array<double, 4> rotMatToQuat(const std::array<double, 9>& R) {
    double w = std::sqrt(std::max(0.0, 1.0 + R[0] + R[4] + R[8])) / 2.0;
    double x = std::sqrt(std::max(0.0, 1.0 + R[0] - R[4] - R[8])) / 2.0;
    double y = std::sqrt(std::max(0.0, 1.0 - R[0] + R[4] - R[8])) / 2.0;
    double z = std::sqrt(std::max(0.0, 1.0 - R[0] - R[4] + R[8])) / 2.0;

    x = std::copysign(x, R[7] - R[5]);
    y = std::copysign(y, R[2] - R[6]);
    z = std::copysign(z, R[3] - R[1]);

    return {x, y, z, w};
}

} // anonymous namespace

ArmNode::ArmNode(const rclcpp::NodeOptions& options, const ArmConfig& config)
    : rclcpp::Node("arm_node", options), arm_(config)
{
    // Declare parameters
    this->declare_parameter("calibration_file", "");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("camera_frame", "camera_link");

    calibration_file_ = this->get_parameter("calibration_file").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    camera_frame_ = this->get_parameter("camera_frame").as_string();

    // Joint state publisher
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "~/joint_states", 10);

    // Timer at 10 Hz
    joint_state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { publishJointState(); });

    // TF static broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    // Load calibration if specified
    if (!calibration_file_.empty()) {
        try {
            loadCalibrationAndBroadcastTF();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to load calibration: %s", e.what());
        }
    }

    setupServices();

    RCLCPP_INFO(this->get_logger(),
        "ArmNode initialized. calibration_file=%s, base_frame=%s, camera_frame=%s",
        calibration_file_.c_str(), base_frame_.c_str(), camera_frame_.c_str());
}

void ArmNode::setupServices() {
    using Trigger = std_srvs::srv::Trigger;

    stop_svc_ = this->create_service<Trigger>(
        "~/stop",
        [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
            try {
                arm_.stop();
                res->success = true;
                res->message = "stopped";
            } catch (const ArmError& e) {
                res->success = false;
                res->message = e.what();
            }
        });
}

void ArmNode::publishJointState() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();

    auto pos = arm_.jointPosition();

    msg.name = {"joint_1", "joint_2", "joint_3",
                "joint_4", "joint_5", "joint_6"};
    msg.position = pos.radians;

    joint_state_pub_->publish(msg);
}

void ArmNode::loadCalibrationAndBroadcastTF() {
    auto calib = loadCalibrationYAML(calibration_file_);
    auto quat = rotMatToQuat(calib.rot_matrix);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->now();
    tf.header.frame_id = base_frame_;
    tf.child_frame_id = camera_frame_;
    tf.transform.translation.x = calib.translation[0];
    tf.transform.translation.y = calib.translation[1];
    tf.transform.translation.z = calib.translation[2];
    tf.transform.rotation.x = quat[0];
    tf.transform.rotation.y = quat[1];
    tf.transform.rotation.z = quat[2];
    tf.transform.rotation.w = quat[3];

    tf_broadcaster_->sendTransform(tf);

    RCLCPP_INFO(this->get_logger(),
        "Calibration TF published: %s -> %s", base_frame_.c_str(), camera_frame_.c_str());
}

} // namespace rm
