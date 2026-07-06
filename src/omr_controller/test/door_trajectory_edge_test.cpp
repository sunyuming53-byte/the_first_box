#include <cmath>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "omr_controller/state_machine/door_trajectory_action.hpp"
#include <rclcpp/rclcpp.hpp>

namespace {

BT::NodeConfig make_config(rclcpp::Node::SharedPtr ros_node) {
    BT::NodeConfig cfg;
    cfg.blackboard = BT::Blackboard::create();
    cfg.blackboard->set("ros_node", ros_node);
    return cfg;
}

// ── EdgeTestNode: mock DoorTrajectoryAction overriding MoveIt2 interface ────

class EdgeTestNode : public omr_controller::DoorTrajectoryAction {
public:
    EdgeTestNode(const std::string& name, const BT::NodeConfig& config)
        : DoorTrajectoryAction(name, config) {}

    using DoorTrajectoryAction::door_objects_added_;
    using DoorTrajectoryAction::getCurrentState;
    using DoorTrajectoryAction::getCurrentWaypointIndex;
    using DoorTrajectoryAction::getWaypoints;
    using DoorTrajectoryAction::setIdleStartTimeForTesting;
    using DoorTrajectoryAction::setJointPositionsForTesting;

    void setPlanResult(bool result) { plan_result_ = result; }
    bool planWasCalled() const { return plan_called_; }
    int planCallCount() const { return plan_call_count_; }

    void setJointHomeResult(bool result) { joint_home_result_ = result; }
    bool jointHomeCalled() const { return joint_home_called_; }

    bool setupDoorCollisionCalled() const { return setup_door_collision_called_; }

    const geometry_msgs::msg::Pose& lastPose() const { return last_pose_; }
    const std::vector<geometry_msgs::msg::Pose>& allPlannedPoses() const {
        return all_planned_poses_;
    }

protected:
    bool planAndExecuteToPose(const geometry_msgs::msg::Pose& target) override {
        plan_called_ = true;
        plan_call_count_++;
        last_pose_ = target;
        all_planned_poses_.push_back(target);
        return plan_result_;
    }

    bool planAndExecuteJointHome() override {
        joint_home_called_ = true;
        return joint_home_result_;
    }

    void setupDoorCollisionObjects() override {
        setup_door_collision_called_ = true;
        door_objects_added_ = true;
    }

private:
    bool plan_result_ = true;
    bool plan_called_ = false;
    int plan_call_count_ = 0;
    geometry_msgs::msg::Pose last_pose_;
    std::vector<geometry_msgs::msg::Pose> all_planned_poses_;

    bool joint_home_result_ = true;
    bool joint_home_called_ = false;

    bool setup_door_collision_called_ = false;
};

void tickUntilState(EdgeTestNode* node, omr_controller::TrajectoryState target,
                    rclcpp::Node* ros_node, int max_ticks = 500) {
    for (int i = 0; i < max_ticks; ++i) {
        rclcpp::spin_some(ros_node->get_node_base_interface());
        auto status = node->executeTick();
        if (node->getCurrentState() == target) return;
        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

bool poseHasNaN(const geometry_msgs::msg::Pose& pose) {
    return std::isnan(pose.position.x) || std::isnan(pose.position.y) ||
           std::isnan(pose.position.z) || std::isnan(pose.orientation.x) ||
           std::isnan(pose.orientation.y) || std::isnan(pose.orientation.z) ||
           std::isnan(pose.orientation.w);
}

}  // namespace

class DoorTrajectoryEdgeTest : public ::testing::Test {
protected:
    void SetUp() override {
        ros_node_ = std::make_shared<rclcpp::Node>("edge_test");
        auto cfg = make_config(ros_node_);
        cfg.input_ports["idle_delay_sec"] = "0.1";
        node_ = std::make_shared<EdgeTestNode>("door_traj", cfg);
        node_->executeTick();
    }

    void TearDown() override { node_.reset(); }

    rclcpp::Node::SharedPtr ros_node_;
    std::shared_ptr<EdgeTestNode> node_;
};

// Test 1: Planning fails for all waypoints → ERROR
TEST_F(DoorTrajectoryEdgeTest, PlanningFailsForAllWaypointsTransitionsToError) {
    auto cfg = make_config(ros_node_);
    cfg.input_ports["idle_delay_sec"] = "0.0";
    cfg.input_ports["theta_max_deg"] = "10.0";
    cfg.input_ports["theta_step_deg"] = "5.0";
    cfg.input_ports["phi_values"] = "0,90";
    auto node = std::make_shared<EdgeTestNode>("door_traj", cfg);
    node->executeTick();

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(node->rosClock()->now() - rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT, ros_node_.get());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);

    node->setPlanResult(false);

    tickUntilState(node.get(), omr_controller::TrajectoryState::ERROR, ros_node_.get());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::ERROR);
    EXPECT_NE(node->getCurrentState(), omr_controller::TrajectoryState::DONE);
}

// Test 2: Extreme θ values produce valid poses
TEST_F(DoorTrajectoryEdgeTest, ExtremeThetaValuesProduceValidPoses) {
    auto cfg = make_config(ros_node_);
    cfg.input_ports["idle_delay_sec"] = "0.0";
    cfg.input_ports["theta_max_deg"] = "180.0";
    cfg.input_ports["theta_step_deg"] = "30.0";
    cfg.input_ports["phi_values"] = "0,45,90";
    auto node = std::make_shared<EdgeTestNode>("door_traj", cfg);
    node->executeTick();

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(node->rosClock()->now() - rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT, ros_node_.get());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);

    const auto& waypoints = node->getWaypoints();
    EXPECT_GT(waypoints.size(), 0u);

    int expected_theta_steps = (180 / 30) + 1;
    int expected_phi_count = 3;
    size_t expected_waypoints = static_cast<size_t>(expected_theta_steps) * expected_phi_count;
    EXPECT_EQ(waypoints.size(), expected_waypoints);

    for (size_t i = 0; i < waypoints.size(); ++i) {
        EXPECT_FALSE(poseHasNaN(waypoints[i])) << "Waypoint " << i << " has NaN component";
    }

    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& o = waypoints[i].orientation;
        double norm_sq = o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w;
        EXPECT_NEAR(norm_sq, 1.0, 1e-6) << "Waypoint " << i << " quaternion not normalised";
    }

    node->setPlanResult(true);
    tickUntilState(node.get(), omr_controller::TrajectoryState::DONE, ros_node_.get(), 1000);
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::DONE);
}

// Test 3: Approach from non-home sets up collision objects
TEST_F(DoorTrajectoryEdgeTest, ApproachFromNonHomeSetsUpCollisionObjects) {
    auto cfg = make_config(ros_node_);
    cfg.input_ports["idle_delay_sec"] = "0.0";
    auto node = std::make_shared<EdgeTestNode>("door_traj", cfg);
    node->executeTick();

    std::vector<double> non_home = {0.5, 0.3, -0.2, 0.1, 0.4, -0.3};
    node->setJointPositionsForTesting(non_home);
    node->setJointHomeResult(true);
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(node->rosClock()->now() - rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node.get(), omr_controller::TrajectoryState::PLAN_APPROACH, ros_node_.get());
    EXPECT_TRUE(node->jointHomeCalled());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLAN_APPROACH);

    tickUntilState(node.get(), omr_controller::TrajectoryState::EXECUTE_APPROACH, ros_node_.get());
    EXPECT_TRUE(node->setupDoorCollisionCalled());
    EXPECT_TRUE(node->door_objects_added_);
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::EXECUTE_APPROACH);

    tickUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT, ros_node_.get());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);
}

// Test 4: Missing hinge_transform defaults to zero
TEST_F(DoorTrajectoryEdgeTest, MissingTransformParamDefaultsToZero) {
    auto cfg = make_config(ros_node_);
    cfg.input_ports["idle_delay_sec"] = "0.1";
    auto node = std::make_shared<EdgeTestNode>("door_traj", cfg);
    node->executeTick();

    const auto& T = node->T_armBase_doorHinge();
    ASSERT_EQ(T.size(), 6u);
    for (double val : T) {
        EXPECT_DOUBLE_EQ(val, 0.0);
    }

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(node->rosClock()->now() - rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node.get(), omr_controller::TrajectoryState::PLAN_APPROACH, ros_node_.get());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLAN_APPROACH);

    tickUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT, ros_node_.get());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);

    const auto& waypoints = node->getWaypoints();
    EXPECT_GT(waypoints.size(), 0u);

    for (size_t i = 0; i < waypoints.size(); ++i) {
        EXPECT_FALSE(poseHasNaN(waypoints[i]));
    }
}

// Test 5: θ locked per plan cycle
TEST_F(DoorTrajectoryEdgeTest, ThetaLockedPerPlanCycle) {
    auto cfg = make_config(ros_node_);
    cfg.input_ports["idle_delay_sec"] = "0.0";
    cfg.input_ports["theta_max_deg"] = "20.0";
    cfg.input_ports["theta_step_deg"] = "10.0";
    cfg.input_ports["phi_values"] = "0,90";
    auto node = std::make_shared<EdgeTestNode>("door_traj", cfg);
    node->executeTick();

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(node->rosClock()->now() - rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node.get(), omr_controller::TrajectoryState::DONE, ros_node_.get(), 1000);
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::DONE);

    const auto& waypoints = node->getWaypoints();
    EXPECT_EQ(waypoints.size(), 6u);
    EXPECT_GE(node->planCallCount(), 6);

    const auto& planned = node->allPlannedPoses();
    ASSERT_GE(planned.size(), waypoints.size());

    size_t approach_offset = planned.size() - waypoints.size();
    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& wp = waypoints[i];
        const auto& pp = planned[approach_offset + i];

        EXPECT_NEAR(wp.position.x, pp.position.x, 1e-9) << "Waypoint " << i;
        EXPECT_NEAR(wp.position.y, pp.position.y, 1e-9) << "Waypoint " << i;
        EXPECT_NEAR(wp.position.z, pp.position.z, 1e-9) << "Waypoint " << i;
    }

    bool poses_differ = false;
    if (waypoints.size() >= 2) {
        double dx = std::abs(waypoints[0].position.x - waypoints[1].position.x);
        double dy = std::abs(waypoints[0].position.y - waypoints[1].position.y);
        double dz = std::abs(waypoints[0].position.z - waypoints[1].position.z);
        poses_differ = (dx > 1e-6 || dy > 1e-6 || dz > 1e-6);
    }
    EXPECT_TRUE(poses_differ);
}
