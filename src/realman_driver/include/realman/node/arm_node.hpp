#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "realman/core/arm.hpp"

namespace rm {

class ArmNode : public rclcpp::Node {
public:
    explicit ArmNode(const rclcpp::NodeOptions& options, const ArmConfig& config);

    Arm& arm() { return arm_; }

private:
    void setupServices();
    void publishJointState();
    void loadCalibrationAndBroadcastTF();

    Arm arm_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_svc_;

    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    std::string calibration_file_;
    std::string base_frame_;
    std::string camera_frame_;
};

} // namespace rm
