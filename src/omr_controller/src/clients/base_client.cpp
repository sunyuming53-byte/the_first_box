#include "omr_controller/clients/base_client.hpp"

#include <cmath>

namespace omr_controller {

BaseClientImpl::BaseClientImpl(rclcpp::Node::SharedPtr node, std::string controller_namespace)
    : node_(std::move(node)), controller_namespace_(std::move(controller_namespace)) {
    const std::string cmdVelTopic = "/" + controller_namespace_ + "/diff_drive_controller/cmd_vel";
    const std::string odomTopic = "/" + controller_namespace_ + "/diff_drive_controller/odom";

    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic, 10);

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odomTopic, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odomCallback(msg); });
}

bool BaseClientImpl::move(double linear_x, double angular_z) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_x;
    twist.angular.z = angular_z;
    cmd_vel_pub_->publish(twist);
    return true;
}

bool BaseClientImpl::stop() {
    geometry_msgs::msg::Twist twist;
    cmd_vel_pub_->publish(twist);
    return true;
}

std::array<double, 3> BaseClientImpl::getPose() const {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    const auto& pos = latest_odom_.pose.pose.position;
    const auto& q = latest_odom_.pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return {pos.x, pos.y, yaw};
}

void BaseClientImpl::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = *msg;
}

}  // namespace omr_controller
