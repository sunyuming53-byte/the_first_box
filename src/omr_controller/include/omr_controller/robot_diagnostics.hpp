#pragma once

#include <cstdint>

#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace omr_controller {

class RobotDiagnostics : public rclcpp::Node {
public:
    explicit RobotDiagnostics(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    double ageSeconds(const rclcpp::Time& stamp) const;
    void setupParameterCallback();
    void setupDiagnostics();
    void summarizeTopic(diagnostic_updater::DiagnosticStatusWrapper& stat, bool enabled,
                        double age_sec, const std::string& ok_message,
                        const std::string& waiting_message, const std::string& stale_message) const;

    void produceArmDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void produceDaisDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void produceM65Diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void produceLioDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void produceOrchestratorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

    bool monitor_arm_{true};
    bool monitor_dais_{true};
    bool monitor_m65_{false};
    bool monitor_lio_{false};
    bool monitor_orchestrator_{false};
    double topic_timeout_sec_{2.0};

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    diagnostic_updater::Updater diagnostics_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m65_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lio_cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lio_estop_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_state_sub_;

    rclcpp::Time last_arm_joint_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_dais_joint_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_m65_odom_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_lio_cloud_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_lio_odom_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_lio_estop_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_task_state_time_{0, 0, RCL_ROS_TIME};

    std::size_t last_joint_count_{0};
    std::size_t arm_joint_count_{0};
    std::size_t dais_joint_count_{0};
    std::uint64_t m65_odom_count_{0};
    std::uint64_t lio_cloud_count_{0};
    std::uint64_t lio_odom_count_{0};
    std::uint64_t task_state_count_{0};
    bool lio_emergency_stop_{false};
    std::string last_task_state_{"UNKNOWN"};
};

}  // namespace omr_controller
