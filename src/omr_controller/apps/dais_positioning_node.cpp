// SPDX-License-Identifier: MIT
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <cmath>
#include <mutex>
#include <string>
#include <vector>

namespace omr_controller {

class DaisPositioningNode : public rclcpp::Node {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    explicit DaisPositioningNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("dais_positioning_node", options)
    {
        declare_parameter("dais_joint_name", "joint_dais");
        declare_parameter("photogate_joint_name", "photogate_gate0_joint");
        declare_parameter("debounce_count", 3);
        declare_parameter("homing_velocity", 0.02);
        declare_parameter("jtc_action",
                          "/dais_joint_trajectory_controller/follow_joint_trajectory");

        dais_joint_name_ = get_parameter("dais_joint_name").as_string();
        pg_joint_name_ = get_parameter("photogate_joint_name").as_string();
        debounce_count_ = get_parameter("debounce_count").as_int();
        homing_velocity_ = get_parameter("homing_velocity").as_double();
        jtc_action_name_ = get_parameter("jtc_action").as_string();

        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                joint_callback(msg);
            });

        ref_pos_pub_ =
            create_publisher<std_msgs::msg::Float64>("/dais/reference_position", 10);

        jtc_client_ =
            rclcpp_action::create_client<FollowJointTrajectory>(this, jtc_action_name_);

        calibrate_srv_ = create_service<std_srvs::srv::Trigger>(
            "/dais/calibrate_reference",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (std::isnan(reference_position_)) {
                    res->success = false;
                    res->message = "no reference detected yet";
                } else {
                    current_offset_ = last_dais_position_ - reference_position_;
                    res->success = true;
                    res->message = "offset=" + std::to_string(current_offset_);
                }
            });

        homing_srv_ = create_service<std_srvs::srv::Trigger>(
            "/dais/start_homing",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                double ref, dais_pos;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (std::isnan(reference_position_)) {
                        res->success = false;
                        res->message =
                            "no reference position — trigger photogate first";
                        return;
                    }
                    ref = reference_position_;
                    dais_pos = last_dais_position_;
                }
                send_homing_trajectory(ref, dais_pos);
                res->success = true;
                res->message = "homing trajectory sent to " + jtc_action_name_;
            });

        updater_ = std::make_unique<diagnostic_updater::Updater>(this);
        updater_->setHardwareID("dais_positioning");
        updater_->add("photogate", this, &DaisPositioningNode::diagnose);
        updater_->force_update();

        RCLCPP_INFO(get_logger(),
                    "DaisPositioningNode started, dais_joint=%s, pg_joint=%s",
                    dais_joint_name_.c_str(), pg_joint_name_.c_str());
    }

private:
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == dais_joint_name_) {
                last_dais_position_ = msg->position[i];
            } else if (msg->name[i] == pg_joint_name_) {
                process_photogate(msg->position[i]);
            }
        }
    }

    void process_photogate(double position) {
        bool blocked = position > 0.5;
        bool prev_stable = photogate_stable_;

        if (blocked != last_blocked_) {
            same_count_ = 0;
            last_blocked_ = blocked;
        } else {
            same_count_++;
            if (same_count_ >= debounce_count_ && blocked != photogate_stable_) {
                photogate_stable_ = blocked;
            }
        }

        if (!prev_stable && photogate_stable_ && blocked &&
            !std::isnan(last_dais_position_)) {
            reference_position_ = last_dais_position_;
            last_edge_time_ = now();

            auto ref_msg = std_msgs::msg::Float64();
            ref_msg.data = reference_position_;
            ref_pos_pub_->publish(ref_msg);

            RCLCPP_INFO(get_logger(), "Photogate triggered at position=%.6f",
                        reference_position_);
        }
    }

    void diagnose(diagnostic_updater::DiagnosticStatusWrapper& stat) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (std::isnan(reference_position_)) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                         "No reference");
        } else {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Referenced");
            stat.add("reference_position", reference_position_);
            stat.add("current_offset", current_offset_);
        }
        stat.add("blocked", photogate_stable_);
        stat.add("dais_position", last_dais_position_);
    }

    void send_homing_trajectory(double ref, double dais_pos) {
        if (!jtc_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "JTC action server not available");
            return;
        }

        auto goal = FollowJointTrajectory::Goal();
        goal.trajectory.joint_names = {dais_joint_name_};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {ref};
        point.velocities = {homing_velocity_};
        point.time_from_start = rclcpp::Duration::from_seconds(
            std::abs(dais_pos - ref) / homing_velocity_);

        goal.trajectory.points = {point};

        auto send_goal_options =
            rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.result_callback =
            [this](const GoalHandleFJT::WrappedResult& result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(), "Homing trajectory completed");
                } else {
                    RCLCPP_WARN(get_logger(), "Homing trajectory failed: %d",
                                static_cast<int>(result.code));
                }
            };

        RCLCPP_INFO(get_logger(), "Sending homing trajectory: ref=%.4f offset=%.4f",
                    ref, current_offset_);
        jtc_client_->async_send_goal(goal, send_goal_options);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ref_pos_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_srv_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr jtc_client_;
    std::unique_ptr<diagnostic_updater::Updater> updater_;

    std::string dais_joint_name_;
    std::string pg_joint_name_;
    std::string jtc_action_name_;
    int debounce_count_ = 3;
    double homing_velocity_ = 0.02;
    mutable std::mutex mutex_;

    double last_dais_position_ = NAN;
    double reference_position_ = NAN;
    double current_offset_ = 0.0;
    bool last_blocked_ = false;
    bool photogate_stable_ = false;
    int same_count_ = 0;

    rclcpp::Time last_edge_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace omr_controller

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<omr_controller::DaisPositioningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
