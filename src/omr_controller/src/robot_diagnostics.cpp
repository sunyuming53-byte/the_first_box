#include "omr_controller/robot_diagnostics.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

#include <cctype>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace omr_controller {

namespace {

constexpr double kDefaultTopicTimeoutSec = 2.0;

bool containsToken(const std::string& text, const std::vector<std::string>& tokens) {
    return std::any_of(tokens.begin(), tokens.end(), [&text](const std::string& token) {
        return text.find(token) != std::string::npos;
    });
}

bool isArmJointName(const std::string& name) {
    constexpr std::size_t kJointPrefixSize = 5;
    if (name.rfind("joint", 0) != 0 || name.size() <= kJointPrefixSize) {
        return false;
    }
    return std::all_of(name.begin() + kJointPrefixSize, name.end(),
                       [](unsigned char ch) { return std::isdigit(ch) != 0; });
}

}  // namespace

RobotDiagnostics::RobotDiagnostics(const rclcpp::NodeOptions& options)
    : Node("robot_diagnostics", options), diagnostics_(this) {
    monitor_arm_ = declare_parameter("monitor_arm", true);
    monitor_dais_ = declare_parameter("monitor_dais", true);
    monitor_m65_ = declare_parameter("monitor_m65", false);
    monitor_lio_ = declare_parameter("monitor_lio", false);
    monitor_orchestrator_ = declare_parameter("monitor_orchestrator", false);
    topic_timeout_sec_ = declare_parameter("topic_timeout_sec", kDefaultTopicTimeoutSec);

    setupParameterCallback();

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            last_joint_count_ = msg->name.size();

            const auto stamp = this->now();
            std::size_t arm_matches = 0;
            std::size_t dais_matches = 0;
            for (const auto& name : msg->name) {
                if (name == "joint_dais" || containsToken(name, {"dais", "D-AIS", "motor"})) {
                    dais_matches++;
                } else if (isArmJointName(name) ||
                           containsToken(name, {"arm", "shoulder", "elbow", "wrist"})) {
                    arm_matches++;
                }
            }

            if (arm_matches > 0) {
                arm_joint_count_ = arm_matches;
                last_arm_joint_time_ = stamp;
            }
            if (dais_matches > 0) {
                dais_joint_count_ = dais_matches;
                last_dais_joint_time_ = stamp;
            }
        });

    m65_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/m65_controller_manager/diff_drive_controller/odom", rclcpp::QoS(20),
        [this](const nav_msgs::msg::Odometry::SharedPtr) {
            last_m65_odom_time_ = this->now();
            m65_odom_count_++;
        });

    lio_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr) {
            last_lio_cloud_time_ = this->now();
            lio_cloud_count_++;
        });

    lio_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/lio/odom", rclcpp::QoS(20), [this](const nav_msgs::msg::Odometry::SharedPtr) {
            last_lio_odom_time_ = this->now();
            lio_odom_count_++;
        });

    lio_estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/lio/emergency_stop", rclcpp::QoS(5), [this](const std_msgs::msg::Bool::SharedPtr msg) {
            last_lio_estop_time_ = this->now();
            lio_emergency_stop_ = msg->data;
        });

    task_state_sub_ = create_subscription<std_msgs::msg::String>(
        "/task_state", rclcpp::QoS(20), [this](const std_msgs::msg::String::SharedPtr msg) {
            last_task_state_time_ = this->now();
            last_task_state_ = msg->data;
            task_state_count_++;
        });

    setupDiagnostics();

    RCLCPP_INFO(get_logger(),
                "RobotDiagnostics started: arm=%s dais=%s m65=%s lio=%s orchestrator=%s "
                "timeout=%.2fs",
                monitor_arm_ ? "true" : "false", monitor_dais_ ? "true" : "false",
                monitor_m65_ ? "true" : "false", monitor_lio_ ? "true" : "false",
                monitor_orchestrator_ ? "true" : "false", topic_timeout_sec_);
}

double RobotDiagnostics::ageSeconds(const rclcpp::Time& stamp) const {
    if (stamp.nanoseconds() == 0) {
        return -1.0;
    }
    return (this->now() - stamp).seconds();
}

void RobotDiagnostics::setupParameterCallback() {
    parameter_callback_handle_ =
        add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            for (const auto& param : params) {
                const auto& name = param.get_name();
                if (name == "topic_timeout_sec") {
                    if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
                        param.as_double() <= 0.0) {
                        result.successful = false;
                        result.reason = "topic_timeout_sec must be a positive double";
                        return result;
                    }
                } else if ((name == "monitor_arm" || name == "monitor_dais" ||
                            name == "monitor_m65" || name == "monitor_lio" ||
                            name == "monitor_orchestrator") &&
                           param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                    result.successful = false;
                    result.reason = name + " must be a bool";
                    return result;
                } else if (name != "monitor_arm" && name != "monitor_dais" &&
                           name != "monitor_m65" && name != "monitor_lio" &&
                           name != "monitor_orchestrator") {
                    result.successful = false;
                    result.reason = name + " is not supported for runtime update";
                    return result;
                }
            }

            for (const auto& param : params) {
                const auto& name = param.get_name();
                if (name == "monitor_arm")
                    monitor_arm_ = param.as_bool();
                else if (name == "monitor_dais")
                    monitor_dais_ = param.as_bool();
                else if (name == "monitor_m65")
                    monitor_m65_ = param.as_bool();
                else if (name == "monitor_lio")
                    monitor_lio_ = param.as_bool();
                else if (name == "monitor_orchestrator")
                    monitor_orchestrator_ = param.as_bool();
                else if (name == "topic_timeout_sec")
                    topic_timeout_sec_ = param.as_double();
            }

            return result;
        });
}

void RobotDiagnostics::setupDiagnostics() {
    diagnostics_.setHardwareID("omr_robot");
    diagnostics_.add("Arm", this, &RobotDiagnostics::produceArmDiagnostics);
    diagnostics_.add("D-AIS", this, &RobotDiagnostics::produceDaisDiagnostics);
    diagnostics_.add("M65", this, &RobotDiagnostics::produceM65Diagnostics);
    diagnostics_.add("LIO", this, &RobotDiagnostics::produceLioDiagnostics);
    diagnostics_.add("Orchestrator", this, &RobotDiagnostics::produceOrchestratorDiagnostics);
    diagnostics_timer_ =
        create_wall_timer(std::chrono::seconds(1), [this]() { diagnostics_.force_update(); });
}

void RobotDiagnostics::summarizeTopic(diagnostic_updater::DiagnosticStatusWrapper& stat,
                                      bool enabled, double age_sec, const std::string& ok_message,
                                      const std::string& waiting_message,
                                      const std::string& stale_message) const {
    if (!enabled) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "monitor disabled by launch");
    } else if (age_sec < 0.0) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, waiting_message);
    } else if (age_sec > topic_timeout_sec_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, stale_message);
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, ok_message);
    }

    stat.add("monitoring_enabled", enabled ? "true" : "false");
    stat.add("topic_timeout_sec", topic_timeout_sec_);
    stat.add("age_sec", age_sec);
}

void RobotDiagnostics::produceArmDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const double age = ageSeconds(last_arm_joint_time_);
    summarizeTopic(stat, monitor_arm_, age, "arm joint states active",
                   "waiting for arm joint states", "arm joint states stale");
    stat.add("topic", "/joint_states");
    stat.add("matched_joint_count", arm_joint_count_);
    stat.add("last_joint_count", last_joint_count_);
}

void RobotDiagnostics::produceDaisDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const double age = ageSeconds(last_dais_joint_time_);
    summarizeTopic(stat, monitor_dais_, age, "D-AIS joint states active",
                   "waiting for D-AIS joint states", "D-AIS joint states stale");
    stat.add("topic", "/joint_states");
    stat.add("matched_joint_count", dais_joint_count_);
    stat.add("expected_joint", "joint_dais");
}

void RobotDiagnostics::produceM65Diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const double age = ageSeconds(last_m65_odom_time_);
    summarizeTopic(stat, monitor_m65_, age, "M65 odometry active", "waiting for M65 odometry",
                   "M65 odometry stale");
    stat.add("topic", "/m65_controller_manager/diff_drive_controller/odom");
    stat.add("message_count", m65_odom_count_);
}

void RobotDiagnostics::produceLioDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const double cloud_age = ageSeconds(last_lio_cloud_time_);
    const double odom_age = ageSeconds(last_lio_odom_time_);
    const bool cloud_ok = cloud_age >= 0.0 && cloud_age <= topic_timeout_sec_;
    const bool odom_ok = odom_age >= 0.0 && odom_age <= topic_timeout_sec_;

    if (!monitor_lio_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "monitor disabled by launch");
    } else if (cloud_age < 0.0 && odom_age < 0.0) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "waiting for LIO topics");
    } else if (!cloud_ok || !odom_ok) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "LIO topic stale");
    } else if (lio_emergency_stop_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "LIO emergency stop active");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "LIO topics active");
    }

    stat.add("monitoring_enabled", monitor_lio_ ? "true" : "false");
    stat.add("cloud_topic", "/cloud_registered");
    stat.add("odom_topic", "/lio/odom");
    stat.add("estop_topic", "/lio/emergency_stop");
    stat.add("cloud_age_sec", cloud_age);
    stat.add("odom_age_sec", odom_age);
    stat.add("estop_age_sec", ageSeconds(last_lio_estop_time_));
    stat.add("cloud_count", lio_cloud_count_);
    stat.add("odom_count", lio_odom_count_);
    stat.add("emergency_stop", lio_emergency_stop_ ? "true" : "false");
}

void RobotDiagnostics::produceOrchestratorDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
    const double age = ageSeconds(last_task_state_time_);
    summarizeTopic(stat, monitor_orchestrator_, age, "orchestrator task state active",
                   "waiting for orchestrator task state", "orchestrator task state stale");
    stat.add("topic", "/task_state");
    stat.add("message_count", task_state_count_);
    stat.add("last_task_state", last_task_state_);
}

}  // namespace omr_controller
