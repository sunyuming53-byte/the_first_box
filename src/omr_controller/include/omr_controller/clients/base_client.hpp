#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace omr_controller {

class BaseClient {
public:
    virtual ~BaseClient() = default;

    virtual bool move(double linear_x, double angular_z) = 0;
    virtual bool stop() = 0;
    virtual std::array<double, 3> getPose() const = 0;
};

class BaseClientImpl : public BaseClient {
public:
    explicit BaseClientImpl(rclcpp::Node::SharedPtr node,
                            std::string controller_namespace = "m65_controller_manager");

    bool move(double linear_x, double angular_z) override;
    bool stop() override;
    std::array<double, 3> getPose() const override;

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    std::string controller_namespace_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    mutable std::mutex odom_mutex_;
    mutable nav_msgs::msg::Odometry latest_odom_;
};

}  // namespace omr_controller
