#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose.hpp>
#include <opencv2/core.hpp>

namespace omr_controller {

/// Convert a 4×4 homogeneous transformation matrix (CV_64F) to a ROS2 Pose.
///
/// Extracts the 3×3 rotation from the top-left block and converts it to a
/// quaternion via tf2::Matrix3x3::getRotation() — no RPY/Euler intermediate,
/// avoiding gimbal lock. Translation is taken from column 3.
///
/// @param T  4×4 homogeneous matrix in row-major OpenCV layout.
/// @return   geometry_msgs::msg::Pose with position and orientation populated.
inline geometry_msgs::msg::Pose homogeneous_to_pose(const cv::Mat& T) {
    // Extract 3×3 rotation from top-left
    cv::Mat R = T(cv::Rect(0, 0, 3, 3));

    // Build tf2::Matrix3x3
    tf2::Matrix3x3 rot(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                       R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                       R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));

    // Convert to quaternion (no RPY/Euler intermediate — avoids gimbal lock)
    tf2::Quaternion q;
    rot.getRotation(q);
    q.normalize();

    geometry_msgs::msg::Pose pose;
    pose.position.x = T.at<double>(0, 3);
    pose.position.y = T.at<double>(1, 3);
    pose.position.z = T.at<double>(2, 3);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

}  // namespace omr_controller
