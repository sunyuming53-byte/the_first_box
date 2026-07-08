#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

namespace omr_lio {

class EstopperNode : public rclcpp::Node {
public:
    explicit EstopperNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_estop_;

    double stop_distance_{0.3};
    int min_points_in_zone_{5};
    bool front_only_{true};
};

}  // namespace omr_lio
