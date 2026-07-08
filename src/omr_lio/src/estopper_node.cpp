#include "omr_lio/estopper_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace omr_lio {

EstopperNode::EstopperNode(const rclcpp::NodeOptions& options) : Node("estopper_node", options) {
    // ── Parameters ──
    stop_distance_ = this->declare_parameter<double>("stop_distance", 0.3);
    min_points_in_zone_ = this->declare_parameter<int>("min_points_in_zone", 5);
    front_only_ = this->declare_parameter<bool>("front_only", true);

    // ── Subscriber ──
    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", rclcpp::QoS(5),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloud_callback(msg); });

    // ── Publisher ──
    pub_estop_ = this->create_publisher<std_msgs::msg::Bool>("/lio/emergency_stop", 1);

    RCLCPP_INFO(this->get_logger(),
                "EstopperNode started — stop_distance=%.2f min_points=%d front_only=%s",
                stop_distance_, min_points_in_zone_, front_only_ ? "true" : "false");
}

void EstopperNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Count points inside the danger zone
    int dangerous_points = 0;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const float x = *iter_x;
        const float y = *iter_y;

        // If front_only, restrict detection to front half of the robot
        if (front_only_ && (x <= 0.0f || std::abs(y) >= 1.0f)) {
            continue;
        }

        // Check if point is within the stop distance
        const float dist = std::sqrt(x * x + y * y + (*iter_z) * (*iter_z));
        if (dist < static_cast<float>(stop_distance_)) {
            ++dangerous_points;
            if (dangerous_points >= min_points_in_zone_) {
                break;
            }
        }
    }

    auto estop_msg = std_msgs::msg::Bool();
    estop_msg.data = (dangerous_points >= min_points_in_zone_);
    pub_estop_->publish(estop_msg);
}

}  // namespace omr_lio
