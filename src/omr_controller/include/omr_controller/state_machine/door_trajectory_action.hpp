#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "omr_controller/door_math.hpp"
#include "omr_controller/geometry_utils.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace omr_controller {

/// @brief States of the trajectory sequencing state machine.
enum class TrajectoryState {
    IDLE,                ///< Waiting for start signal.
    APPROACH_HOME,       ///< Verifying arm is at home position.
    PLAN_APPROACH,       ///< Planning + executing approach to first waypoint.
    EXECUTE_APPROACH,    ///< Verifying approach completion.
    PREPARE_WAYPOINTS,   ///< Pre-computing all (θ, φ) waypoint poses.
    PLANNING_WAYPOINT,   ///< Planning + executing a single waypoint.
    EXECUTING_WAYPOINT,  ///< Verifying waypoint completion.
    WAITING_COMPLETION,  ///< Checking if waypoint reached; advancing index.
    DONE,                ///< Trajectory sequence complete.
    ERROR                ///< Fatal error; sequence aborted.
};

/// @brief BT.CPP v4 StatefulActionNode that generates and executes a
///        collision-aware door-opening trajectory via MoveIt2.
///
/// Drives the (θ, φ) schedule, calls planAndExecuteToPose() for each waypoint,
/// and returns SUCCESS on completion or FAILURE on error.
///
/// The node obtains an rclcpp::Node::SharedPtr from the blackboard key
/// "ros_node", which is used to subscribe to /joint_states and construct
/// a MoveGroupInterface.
class DoorTrajectoryAction : public BT::StatefulActionNode {
public:
    DoorTrajectoryAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // ── Public accessors (for testing) ───────────────────────────────────────

    double r() const { return r_; }
    double L() const { return L_; }
    double h() const { return h_; }
    const std::vector<double>& T_armBase_doorHinge() const { return T_armBase_doorHinge_; }
    double theta_step_deg() const { return theta_step_deg_; }
    double theta_max_deg() const { return theta_max_deg_; }
    const std::vector<double>& phi_values_deg() const { return phi_values_deg_; }
    const std::vector<double>& omega_values_deg() const { return omega_values_deg_; }
    const std::vector<double>& home_joints() const { return home_joints_; }
    double joint_state_tolerance() const { return joint_state_tolerance_; }
    double idle_delay_sec() const { return idle_delay_sec_; }
    const std::vector<double>& door_panel_size() const { return door_panel_size_; }
    double door_frame_radius() const { return door_frame_radius_; }
    const std::string& planning_frame() const { return planning_frame_; }

    // ── Test utilities (public for test access) ───────────────────────────

    /// @brief Replace the internal MoveGroupInterface (for test mocking).
    void
    setMoveGroupForTesting(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg) {
        move_group_ = std::move(mg);
    }

    /// @brief Directly set cached joint positions (for test injection).
    void setJointPositionsForTesting(const std::vector<double>& positions) {
        std::lock_guard<std::mutex> lock(joints_mutex_);
        current_joints_ = positions;
    }

    /// @brief Read cached joint positions (for test assertions).
    std::vector<double> getJointPositionsForTesting() const {
        std::lock_guard<std::mutex> lock(joints_mutex_);
        return current_joints_;
    }

    /// @brief Get current state machine state (for test assertions).
    TrajectoryState getCurrentState() const { return current_state_; }

    /// @brief Get pre-computed waypoints (for test assertions).
    const std::vector<geometry_msgs::msg::Pose>& getWaypoints() const { return waypoints_; }

    /// @brief Get current waypoint index (for test assertions).
    size_t getCurrentWaypointIndex() const { return current_waypoint_; }

    /// @brief Inject idle start time (for test acceleration).
    void setIdleStartTimeForTesting(const rclcpp::Time& t) { idle_start_time_ = t; }

    /// @brief Access the blackboard ros_node (for test usage).
    rclcpp::Node::SharedPtr rosNode() const { return ros_node_; }

    /// @brief Get the blackboard node's clock (for test assertions).
    rclcpp::Clock::SharedPtr rosClock() const { return ros_node_->get_clock(); }

    // ── Collision object builders (public for test access) ────────────────

    /// @brief Build the door_panel CollisionObject message.
    moveit_msgs::msg::CollisionObject buildDoorPanelMsg(double theta_rad,
                                                        const cv::Mat& T_base_hinge) const;

    /// @brief Build the door_frame CollisionObject message.
    moveit_msgs::msg::CollisionObject buildDoorFrameMsg(const cv::Mat& T_base_hinge) const;

    /// @brief Compute a single waypoint pose for given (θ, φ, ω) in radians.
    ///        Uses the gimbal-joint three-parameter kinematic model.
    geometry_msgs::msg::Pose computePoseForThetaPhiOmega(double theta_rad, double phi_rad,
                                                         double omega_rad) const;

protected:
    /// @brief MoveGroupInterface — lazy-initialised on first planAndExecuteToPose().
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    /// @brief PlanningSceneInterface — lazy-initialised, for managing door
    ///        collision objects.
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

    /// @brief Whether door collision objects have been added to the scene.
    bool door_objects_added_{false};

    std::vector<double> current_joints_;
    mutable std::mutex joints_mutex_;

    // ── State machine members ────────────────────────────────────────────────

    TrajectoryState current_state_ = TrajectoryState::IDLE;
    rclcpp::Time idle_start_time_;
    std::vector<geometry_msgs::msg::Pose> waypoints_;
    std::vector<double> waypoint_thetas_;
    size_t current_waypoint_ = 0;
    geometry_msgs::msg::Pose approach_pose_;

    // ── MoveIt2 helpers (virtual for test mocking) ───────────────────────────

    /// @brief Ensure move_group_ is initialised (called on first use).
    void ensureMoveGroup();

    /// @brief Handler for /joint_states subscription.
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    /// @brief Poll current_joints_ until all 6 joints are within tolerance of
    ///        the target or timeout_sec elapses.
    bool waitForCompletion(const std::vector<double>& target_joints, double timeout_sec);

    // ── Virtual motion methods (overridable for test mocking) ───────────────

    virtual bool planAndExecuteToPose(const geometry_msgs::msg::Pose& target);

    /// @brief Plan and execute a joint-space move to the home position.
    virtual bool planAndExecuteJointHome();

    /// @brief Set up door collision objects (panel + frame) in the planning
    ///        scene. Idempotent — only adds objects once per node lifetime.
    virtual void setupDoorCollisionObjects();

    /// @brief Update the door panel pose as a function of opening angle θ.
    void updateDoorPose(double theta_rad);

private:
    /// @brief Check whether all 6 cached joint positions are within
    ///        joint_state_tolerance_ of home_joints_.
    bool isAtHome() const;

    /// @brief Parse a comma-separated string of doubles.
    static std::vector<double> parseDoubles(const std::string& str);

    /// @brief Convert state enum to string for logging.
    static const char* stateName(TrajectoryState s);

    // ── Parameters (read from ports) ─────────────────────────────────────
    double r_;
    double L_;
    double h_;
    std::vector<double> T_armBase_doorHinge_;
    double theta_step_deg_;
    double theta_max_deg_;
    std::vector<double> phi_values_deg_;
    std::vector<double> omega_values_deg_;
    std::vector<double> home_joints_;
    double joint_state_tolerance_;
    double idle_delay_sec_;
    std::vector<double> door_panel_size_;
    double door_frame_radius_;
    std::string planning_frame_;

    /// @brief Cached 4×4 homogeneous transform from base_link to door hinge frame.
    ///        Computed once in onStart() from hinge_transform port.
    cv::Mat T_base_hinge_cache_;

    // ── ROS interfaces (from blackboard) ─────────────────────────────────
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

}  // namespace omr_controller
