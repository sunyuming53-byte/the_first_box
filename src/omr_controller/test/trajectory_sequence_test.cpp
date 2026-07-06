#include "omr_controller/door_trajectory_node.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace {

class TrajectorySequenceTestNode : public omr_controller::DoorTrajectoryNode {
public:
    explicit TrajectorySequenceTestNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : DoorTrajectoryNode(options) {}

    using DoorTrajectoryNode::getCurrentState;
    using DoorTrajectoryNode::getWaypoints;
    using DoorTrajectoryNode::getCurrentWaypointIndex;
    using DoorTrajectoryNode::setJointPositionsForTesting;
    using DoorTrajectoryNode::setIdleStartTimeForTesting;
    using DoorTrajectoryNode::door_objects_added_;

    void setPlanResult(bool result) { plan_result_ = result; }
    bool planWasCalled() const { return plan_called_; }
    int planCallCount() const { return plan_call_count_; }

    void setJointHomeResult(bool result) { joint_home_result_ = result; }
    bool jointHomeCalled() const { return joint_home_called_; }

    bool setupDoorCollisionCalled() const {
        return setup_door_collision_called_;
    }

protected:
    bool planAndExecuteToPose(
        const geometry_msgs::msg::Pose& /*target*/) override {
        plan_called_ = true;
        plan_call_count_++;
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
    bool joint_home_result_ = true;
    bool joint_home_called_ = false;
    bool setup_door_collision_called_ = false;
};

void spinUntilState(rclcpp::Node* node,
                    omr_controller::TrajectoryState target,
                    std::chrono::milliseconds timeout = std::chrono::seconds(2)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    auto* test_node = dynamic_cast<TrajectorySequenceTestNode*>(node);
    while (std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node->get_node_base_interface());
        if (test_node && test_node->getCurrentState() == target) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

}  // namespace

class TrajectorySequenceTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::NodeOptions opts;
        opts.append_parameter_override("idle_delay_sec", 0.1);
        opts.append_parameter_override("tick_rate", 100.0);
        node_ = std::make_shared<TrajectorySequenceTestNode>(opts);
    }

    void TearDown() override { node_.reset(); }

    std::shared_ptr<TrajectorySequenceTestNode> node_;
};

// ── Test 1: IDLE → APPROACH_HOME → PLAN_APPROACH ───────────────────────────

TEST_F(TrajectorySequenceTest, IdleToApproachHomeToPlanApproach) {
    node_->setJointPositionsForTesting(node_->home_joints());
    node_->setPlanResult(true);

    EXPECT_EQ(node_->getCurrentState(),
              omr_controller::TrajectoryState::IDLE);

    node_->setIdleStartTimeForTesting(
        node_->get_clock()->now() - rclcpp::Duration::from_seconds(1.0));

    spinUntilState(node_.get(), omr_controller::TrajectoryState::PLAN_APPROACH);
    EXPECT_EQ(node_->getCurrentState(),
              omr_controller::TrajectoryState::PLAN_APPROACH);

    spinUntilState(node_.get(), omr_controller::TrajectoryState::EXECUTE_APPROACH);
    EXPECT_TRUE(node_->planWasCalled());
    EXPECT_EQ(node_->getCurrentState(),
              omr_controller::TrajectoryState::EXECUTE_APPROACH);
}

// ── Test 2: PREPARE_WAYPOINTS computes correct count ────────────────────────

TEST_F(TrajectorySequenceTest, PrepareWaypointsCount) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("idle_delay_sec", 0.0);
    opts.append_parameter_override("tick_rate", 100.0);
    opts.append_parameter_override("theta_step_deg", 10.0);
    opts.append_parameter_override("theta_max_deg", 30.0);
    opts.append_parameter_override("phi_values_deg",
                                   std::vector<double>({0.0, 45.0, 90.0}));
    auto node = std::make_shared<TrajectorySequenceTestNode>(opts);

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(
        node->get_clock()->now() - rclcpp::Duration::from_seconds(1.0));

    spinUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);
    EXPECT_EQ(node->getCurrentState(),
              omr_controller::TrajectoryState::PLANNING_WAYPOINT);

    const auto& waypoints = node->getWaypoints();
    double theta_min = 0.0;
    double theta_max = 30.0;
    double theta_step = 10.0;
    int expected_theta_steps =
        static_cast<int>((theta_max - theta_min) / theta_step) + 1;
    int expected_phi_count = 3;
    size_t expected_waypoints =
        static_cast<size_t>(expected_theta_steps) * expected_phi_count;

    EXPECT_EQ(waypoints.size(), expected_waypoints)
        << expected_theta_steps << " theta steps x "
        << expected_phi_count << " phi values";
}

// ── Test 3: Single waypoint full cycle ─────────────────────────────────────

TEST_F(TrajectorySequenceTest, SingleWaypointFullCycle) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("idle_delay_sec", 0.0);
    opts.append_parameter_override("tick_rate", 100.0);
    opts.append_parameter_override("theta_max_deg", 0.0);
    opts.append_parameter_override("phi_values_deg",
                                   std::vector<double>({0.0}));
    auto node = std::make_shared<TrajectorySequenceTestNode>(opts);

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(
        node->get_clock()->now() - rclcpp::Duration::from_seconds(1.0));

    spinUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);
    EXPECT_EQ(node->getCurrentState(),
              omr_controller::TrajectoryState::PLANNING_WAYPOINT);
    EXPECT_EQ(node->getWaypoints().size(), 1u);

    spinUntilState(node.get(), omr_controller::TrajectoryState::DONE);
    EXPECT_EQ(node->getCurrentState(),
              omr_controller::TrajectoryState::DONE);
}

// ── Test 4: Approach plan failure → ERROR ─────────────────────────────────

TEST_F(TrajectorySequenceTest, PlanFailureTransitionsToError) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("idle_delay_sec", 0.0);
    opts.append_parameter_override("tick_rate", 100.0);
    opts.append_parameter_override("theta_max_deg", 10.0);
    auto node = std::make_shared<TrajectorySequenceTestNode>(opts);

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(false);
    node->setIdleStartTimeForTesting(
        node->get_clock()->now() - rclcpp::Duration::from_seconds(1.0));

    spinUntilState(node.get(), omr_controller::TrajectoryState::ERROR);
    EXPECT_EQ(node->getCurrentState(),
              omr_controller::TrajectoryState::ERROR);
}

// ── Test 5: Waypoint plan failure mid-sequence → ERROR ─────────────────────

TEST_F(TrajectorySequenceTest, WaypointFailureTransitionsToError) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("idle_delay_sec", 0.0);
    opts.append_parameter_override("tick_rate", 100.0);
    opts.append_parameter_override("theta_max_deg", 10.0);
    opts.append_parameter_override("phi_values_deg",
                                   std::vector<double>({0.0}));
    auto node = std::make_shared<TrajectorySequenceTestNode>(opts);

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(
        node->get_clock()->now() - rclcpp::Duration::from_seconds(1.0));

    spinUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);
    EXPECT_EQ(node->getCurrentState(),
              omr_controller::TrajectoryState::PLANNING_WAYPOINT);

    node->setPlanResult(false);

    spinUntilState(node.get(), omr_controller::TrajectoryState::ERROR);
    EXPECT_EQ(node->getCurrentState(),
              omr_controller::TrajectoryState::ERROR);
    EXPECT_EQ(node->getCurrentWaypointIndex(), 0u);
}

// ── Test 6: Multiple waypoints advance index correctly ─────────────────────

TEST_F(TrajectorySequenceTest, MultipleWaypointsAdvanceIndex) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("idle_delay_sec", 0.0);
    opts.append_parameter_override("tick_rate", 100.0);
    opts.append_parameter_override("theta_max_deg", 5.0);
    opts.append_parameter_override("theta_step_deg", 5.0);
    opts.append_parameter_override("phi_values_deg",
                                   std::vector<double>({0.0, 90.0}));
    auto node = std::make_shared<TrajectorySequenceTestNode>(opts);

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(
        node->get_clock()->now() - rclcpp::Duration::from_seconds(1.0));

    spinUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);
    EXPECT_EQ(node->getCurrentState(),
              omr_controller::TrajectoryState::PLANNING_WAYPOINT);
    EXPECT_EQ(node->getWaypoints().size(), 4u);
    EXPECT_EQ(node->getCurrentWaypointIndex(), 0u);

    spinUntilState(node.get(), omr_controller::TrajectoryState::DONE);
    EXPECT_EQ(node->getCurrentState(),
              omr_controller::TrajectoryState::DONE);
    EXPECT_GE(node->planCallCount(), 4);
}

// ── Test 7: Approach from non-home triggers joint-space home move ──────────

TEST_F(TrajectorySequenceTest, ApproachFromNonHomeTriggersJointHome) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("idle_delay_sec", 0.0);
    opts.append_parameter_override("tick_rate", 100.0);
    auto node = std::make_shared<TrajectorySequenceTestNode>(opts);

    std::vector<double> non_home = {0.5, 0.3, -0.2, 0.1, 0.4, -0.3};
    node->setJointPositionsForTesting(non_home);
    node->setJointHomeResult(true);
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(
        node->get_clock()->now() - rclcpp::Duration::from_seconds(1.0));

    spinUntilState(node.get(), omr_controller::TrajectoryState::PLAN_APPROACH);
    EXPECT_TRUE(node->jointHomeCalled());
    EXPECT_EQ(node->getCurrentState(),
              omr_controller::TrajectoryState::PLAN_APPROACH);
}

// ── Test 8: Approach from home skips joint home command ────────────────────

TEST_F(TrajectorySequenceTest, ApproachFromHomeSkipsJointHome) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("idle_delay_sec", 0.0);
    opts.append_parameter_override("tick_rate", 100.0);
    auto node = std::make_shared<TrajectorySequenceTestNode>(opts);

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(
        node->get_clock()->now() - rclcpp::Duration::from_seconds(1.0));

    spinUntilState(node.get(), omr_controller::TrajectoryState::PLAN_APPROACH);
    EXPECT_FALSE(node->jointHomeCalled())
        << "joint home move must NOT be called when arm is already at home";
    EXPECT_EQ(node->getCurrentState(),
              omr_controller::TrajectoryState::PLAN_APPROACH);
}

// ── Test 9: PLAN_APPROACH sets up door collision objects ───────────────────

TEST_F(TrajectorySequenceTest, PlanApproachSetsUpCollisionObjects) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("idle_delay_sec", 0.0);
    opts.append_parameter_override("tick_rate", 100.0);
    auto node = std::make_shared<TrajectorySequenceTestNode>(opts);

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(
        node->get_clock()->now() - rclcpp::Duration::from_seconds(1.0));

    spinUntilState(node.get(),
                   omr_controller::TrajectoryState::EXECUTE_APPROACH);
    EXPECT_TRUE(node->setupDoorCollisionCalled());
    EXPECT_TRUE(node->door_objects_added_);
    EXPECT_EQ(node->getCurrentState(),
              omr_controller::TrajectoryState::EXECUTE_APPROACH);
}

// ── Test 10: Non-home approach failure → ERROR ────────────────────────────

TEST_F(TrajectorySequenceTest, NonHomeApproachFailureTransitionsToError) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("idle_delay_sec", 0.0);
    opts.append_parameter_override("tick_rate", 100.0);
    auto node = std::make_shared<TrajectorySequenceTestNode>(opts);

    std::vector<double> non_home = {0.5, 0.3, -0.2, 0.1, 0.4, -0.3};
    node->setJointPositionsForTesting(non_home);
    node->setJointHomeResult(false);  // home approach fails
    node->setIdleStartTimeForTesting(
        node->get_clock()->now() - rclcpp::Duration::from_seconds(1.0));

    spinUntilState(node.get(), omr_controller::TrajectoryState::ERROR);
    EXPECT_TRUE(node->jointHomeCalled());
    EXPECT_EQ(node->getCurrentState(),
              omr_controller::TrajectoryState::ERROR);
}
