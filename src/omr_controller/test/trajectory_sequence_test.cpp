#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "omr_controller/state_machine/door_trajectory_action.hpp"
#include <rclcpp/rclcpp.hpp>

namespace {

// Helper: create a minimal BT::NodeConfig with ros_node on blackboard.
BT::NodeConfig make_config(rclcpp::Node::SharedPtr ros_node) {
    BT::NodeConfig cfg;
    cfg.blackboard = BT::Blackboard::create();
    cfg.blackboard->set("ros_node", ros_node);
    return cfg;
}

// Test adapter with mocked MoveIt2 methods.
class TrajectorySequenceTestNode : public omr_controller::DoorTrajectoryAction {
public:
    TrajectorySequenceTestNode(const std::string& name, const BT::NodeConfig& config)
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

protected:
    bool planAndExecuteToPose(const geometry_msgs::msg::Pose& /*target*/) override {
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

// Tick the BT node until it reaches the target state or timeout.
void tickUntilState(TrajectorySequenceTestNode* node, omr_controller::TrajectoryState target,
                    rclcpp::Node* ros_node, int max_ticks = 500) {
    for (int i = 0; i < max_ticks; ++i) {
        rclcpp::spin_some(ros_node->get_node_base_interface());
        auto status = node->executeTick();
        if (node->getCurrentState() == target) return;
        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

}  // namespace

class TrajectorySequenceTest : public ::testing::Test {
protected:
    void SetUp() override {
        ros_node_ = std::make_shared<rclcpp::Node>("trajectory_test");
        auto cfg = make_config(ros_node_);
        cfg.input_ports["idle_delay_sec"] = "0.1";
        node_ = std::make_shared<TrajectorySequenceTestNode>("door_traj", cfg);
        node_->executeTick();  // onStart() initialises
    }

    void TearDown() override { node_.reset(); }

    rclcpp::Node::SharedPtr ros_node_;
    std::shared_ptr<TrajectorySequenceTestNode> node_;
};

// ── Test 1: IDLE → APPROACH_HOME → PLAN_APPROACH ──────────────────────────

TEST_F(TrajectorySequenceTest, IdleToApproachHomeToPlanApproach) {
    node_->setJointPositionsForTesting(node_->home_joints());
    node_->setPlanResult(true);

    EXPECT_EQ(node_->getCurrentState(), omr_controller::TrajectoryState::IDLE);

    node_->setIdleStartTimeForTesting(node_->rosClock()->now() -
                                      rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node_.get(), omr_controller::TrajectoryState::PLAN_APPROACH, ros_node_.get());
    EXPECT_EQ(node_->getCurrentState(), omr_controller::TrajectoryState::PLAN_APPROACH);

    tickUntilState(node_.get(), omr_controller::TrajectoryState::EXECUTE_APPROACH, ros_node_.get());
    EXPECT_TRUE(node_->planWasCalled());
    EXPECT_EQ(node_->getCurrentState(), omr_controller::TrajectoryState::EXECUTE_APPROACH);
}

// ── Test 2: PREPARE_WAYPOINTS computes correct count ───────────────────────

TEST_F(TrajectorySequenceTest, PrepareWaypointsCount) {
    auto cfg2 = make_config(ros_node_);
    cfg2.input_ports["idle_delay_sec"] = "0.0";
    cfg2.input_ports["theta_step_deg"] = "10.0";
    cfg2.input_ports["theta_max_deg"] = "30.0";
    cfg2.input_ports["phi_values"] = "0,45,90";
    auto node = std::make_shared<TrajectorySequenceTestNode>("door_traj", cfg2);
    node->executeTick();

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(node->rosClock()->now() - rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT, ros_node_.get());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);

    const auto& waypoints = node->getWaypoints();
    double theta_min = 0.0;
    double theta_max = 30.0;
    double theta_step = 10.0;
    int expected_theta_steps = static_cast<int>((theta_max - theta_min) / theta_step) + 1;
    int expected_phi_count = 3;
    size_t expected_waypoints = static_cast<size_t>(expected_theta_steps) * expected_phi_count;

    EXPECT_EQ(waypoints.size(), expected_waypoints)
        << expected_theta_steps << " theta steps x " << expected_phi_count << " phi values";
}

// ── Test 3: Single waypoint full cycle ─────────────────────────────────────

TEST_F(TrajectorySequenceTest, SingleWaypointFullCycle) {
    auto cfg2 = make_config(ros_node_);
    cfg2.input_ports["idle_delay_sec"] = "0.0";
    cfg2.input_ports["theta_max_deg"] = "0.0";
    cfg2.input_ports["phi_values"] = "0";
    auto node = std::make_shared<TrajectorySequenceTestNode>("door_traj", cfg2);
    node->executeTick();

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(node->rosClock()->now() - rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT, ros_node_.get());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);
    EXPECT_EQ(node->getWaypoints().size(), 1u);

    tickUntilState(node.get(), omr_controller::TrajectoryState::DONE, ros_node_.get());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::DONE);
}

// ── Test 4: Approach plan failure → ERROR ─────────────────────────────────

TEST_F(TrajectorySequenceTest, PlanFailureTransitionsToError) {
    auto cfg2 = make_config(ros_node_);
    cfg2.input_ports["idle_delay_sec"] = "0.0";
    cfg2.input_ports["theta_max_deg"] = "10.0";
    auto node = std::make_shared<TrajectorySequenceTestNode>("door_traj", cfg2);
    node->executeTick();

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(false);
    node->setIdleStartTimeForTesting(node->rosClock()->now() - rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node.get(), omr_controller::TrajectoryState::ERROR, ros_node_.get());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::ERROR);
}

// ── Test 5: Waypoint plan failure mid-sequence → ERROR ─────────────────────

TEST_F(TrajectorySequenceTest, WaypointFailureTransitionsToError) {
    auto cfg2 = make_config(ros_node_);
    cfg2.input_ports["idle_delay_sec"] = "0.0";
    cfg2.input_ports["theta_max_deg"] = "10.0";
    cfg2.input_ports["phi_values"] = "0";
    auto node = std::make_shared<TrajectorySequenceTestNode>("door_traj", cfg2);
    node->executeTick();

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(node->rosClock()->now() - rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT, ros_node_.get());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);

    node->setPlanResult(false);

    tickUntilState(node.get(), omr_controller::TrajectoryState::ERROR, ros_node_.get());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::ERROR);
    EXPECT_EQ(node->getCurrentWaypointIndex(), 0u);
}

// ── Test 6: Multiple waypoints advance index correctly ─────────────────────

TEST_F(TrajectorySequenceTest, MultipleWaypointsAdvanceIndex) {
    auto cfg2 = make_config(ros_node_);
    cfg2.input_ports["idle_delay_sec"] = "0.0";
    cfg2.input_ports["theta_max_deg"] = "5.0";
    cfg2.input_ports["theta_step_deg"] = "5.0";
    cfg2.input_ports["phi_values"] = "0,90";
    auto node = std::make_shared<TrajectorySequenceTestNode>("door_traj", cfg2);
    node->executeTick();

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(node->rosClock()->now() - rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT, ros_node_.get());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);
    EXPECT_EQ(node->getWaypoints().size(), 4u);
    EXPECT_EQ(node->getCurrentWaypointIndex(), 0u);

    tickUntilState(node.get(), omr_controller::TrajectoryState::DONE, ros_node_.get());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::DONE);
    EXPECT_GE(node->planCallCount(), 4);
}

// ── Test 7: Approach from non-home triggers joint-space home move ──────────

TEST_F(TrajectorySequenceTest, ApproachFromNonHomeTriggersJointHome) {
    auto cfg2 = make_config(ros_node_);
    cfg2.input_ports["idle_delay_sec"] = "0.0";
    auto node = std::make_shared<TrajectorySequenceTestNode>("door_traj", cfg2);
    node->executeTick();

    std::vector<double> non_home = {0.5, 0.3, -0.2, 0.1, 0.4, -0.3};
    node->setJointPositionsForTesting(non_home);
    node->setJointHomeResult(true);
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(node->rosClock()->now() - rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node.get(), omr_controller::TrajectoryState::PLAN_APPROACH, ros_node_.get());
    EXPECT_TRUE(node->jointHomeCalled());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLAN_APPROACH);
}

// ── Test 8: Approach from home skips joint home command ────────────────────

TEST_F(TrajectorySequenceTest, ApproachFromHomeSkipsJointHome) {
    auto cfg2 = make_config(ros_node_);
    cfg2.input_ports["idle_delay_sec"] = "0.0";
    auto node = std::make_shared<TrajectorySequenceTestNode>("door_traj", cfg2);
    node->executeTick();

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(node->rosClock()->now() - rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node.get(), omr_controller::TrajectoryState::PLAN_APPROACH, ros_node_.get());
    EXPECT_FALSE(node->jointHomeCalled())
        << "joint home move must NOT be called when arm is already at home";
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLAN_APPROACH);
}

// ── Test 9: PLAN_APPROACH sets up door collision objects ───────────────────

TEST_F(TrajectorySequenceTest, PlanApproachSetsUpCollisionObjects) {
    auto cfg2 = make_config(ros_node_);
    cfg2.input_ports["idle_delay_sec"] = "0.0";
    auto node = std::make_shared<TrajectorySequenceTestNode>("door_traj", cfg2);
    node->executeTick();

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(node->rosClock()->now() - rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node.get(), omr_controller::TrajectoryState::EXECUTE_APPROACH, ros_node_.get());
    EXPECT_TRUE(node->setupDoorCollisionCalled());
    EXPECT_TRUE(node->door_objects_added_);
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::EXECUTE_APPROACH);
}

// ── Test 10: Non-home approach failure → ERROR ────────────────────────────

TEST_F(TrajectorySequenceTest, NonHomeApproachFailureTransitionsToError) {
    auto cfg2 = make_config(ros_node_);
    cfg2.input_ports["idle_delay_sec"] = "0.0";
    auto node = std::make_shared<TrajectorySequenceTestNode>("door_traj", cfg2);
    node->executeTick();

    std::vector<double> non_home = {0.5, 0.3, -0.2, 0.1, 0.4, -0.3};
    node->setJointPositionsForTesting(non_home);
    node->setJointHomeResult(false);  // home approach fails
    node->setIdleStartTimeForTesting(node->rosClock()->now() - rclcpp::Duration::from_seconds(1.0));

    tickUntilState(node.get(), omr_controller::TrajectoryState::ERROR, ros_node_.get());
    EXPECT_TRUE(node->jointHomeCalled());
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::ERROR);
}
