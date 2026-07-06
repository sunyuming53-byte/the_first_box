#include <cmath>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "omr_controller/door_trajectory_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace {

// ── EdgeTestNode: mock DoorTrajectoryNode overriding MoveIt2 interface ──────

class EdgeTestNode : public omr_controller::DoorTrajectoryNode {
public:
    explicit EdgeTestNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : DoorTrajectoryNode(options) {}

    // Expose protected members for test assertions.
    using DoorTrajectoryNode::door_objects_added_;
    using DoorTrajectoryNode::getCurrentState;
    using DoorTrajectoryNode::getCurrentWaypointIndex;
    using DoorTrajectoryNode::getWaypoints;
    using DoorTrajectoryNode::setIdleStartTimeForTesting;
    using DoorTrajectoryNode::setJointPositionsForTesting;

    // ── Plan result injection ─────────────────────────────────────────────
    void setPlanResult(bool result) { plan_result_ = result; }
    bool planWasCalled() const { return plan_called_; }
    int planCallCount() const { return plan_call_count_; }

    // ── Joint-home result injection ───────────────────────────────────────
    void setJointHomeResult(bool result) { joint_home_result_ = result; }
    bool jointHomeCalled() const { return joint_home_called_; }

    // ── Collision mock ────────────────────────────────────────────────────
    bool setupDoorCollisionCalled() const { return setup_door_collision_called_; }

    // ── Pose tracking (for θ-locking test) ────────────────────────────────
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

// ── Spin helper: pump rclcpp until target state or timeout ──────────────────

void spinUntilState(rclcpp::Node* node, omr_controller::TrajectoryState target,
                    std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    auto* test_node = dynamic_cast<EdgeTestNode*>(node);
    while (std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node->get_node_base_interface());
        if (test_node && test_node->getCurrentState() == target) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// ── Helper: check if a pose component is NaN ────────────────────────────────

bool poseHasNaN(const geometry_msgs::msg::Pose& pose) {
    return std::isnan(pose.position.x) || std::isnan(pose.position.y) ||
           std::isnan(pose.position.z) || std::isnan(pose.orientation.x) ||
           std::isnan(pose.orientation.y) || std::isnan(pose.orientation.z) ||
           std::isnan(pose.orientation.w);
}

}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
// DoorTrajectoryEdgeTest — fixture
// ═════════════════════════════════════════════════════════════════════════════

class DoorTrajectoryEdgeTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::NodeOptions opts;
        opts.append_parameter_override("idle_delay_sec", 0.1);
        opts.append_parameter_override("tick_rate", 100.0);
        node_ = std::make_shared<EdgeTestNode>(opts);
    }

    void TearDown() override { node_.reset(); }

    std::shared_ptr<EdgeTestNode> node_;
};

// ═════════════════════════════════════════════════════════════════════════════
// Test 1: Planning fails for all waypoints → ERROR, no crash/hang
// ═════════════════════════════════════════════════════════════════════════════

TEST_F(DoorTrajectoryEdgeTest, PlanningFailsForAllWaypointsTransitionsToError) {
    // Set up a multi-waypoint scenario where ALL plans fail.
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("idle_delay_sec", 0.0);
    opts.append_parameter_override("tick_rate", 100.0);
    opts.append_parameter_override("theta_max_deg", 10.0);
    opts.append_parameter_override("theta_step_deg", 5.0);
    opts.append_parameter_override("phi_values_deg", std::vector<double>({0.0, 90.0}));
    auto node = std::make_shared<EdgeTestNode>(opts);

    node->setJointPositionsForTesting(node->home_joints());
    // Approach plan MUST succeed to reach waypoint phase — fail at waypoints.
    node->setPlanResult(true);  // approach succeeds
    node->setIdleStartTimeForTesting(node->get_clock()->now() -
                                     rclcpp::Duration::from_seconds(1.0));

    // Navigate to PLANNING_WAYPOINT (approach + first waypoint succeed).
    spinUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);

    // Now flip the switch: ALL subsequent plans fail.
    node->setPlanResult(false);

    // The tick at PLANNING_WAYPOINT calls planAndExecuteToPose,
    // which returns false → ERROR.
    spinUntilState(node.get(), omr_controller::TrajectoryState::ERROR, std::chrono::seconds(3));

    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::ERROR)
        << "Node must transition to ERROR when all waypoint plans fail";
    // Sanity: it did not reach DONE.
    EXPECT_NE(node->getCurrentState(), omr_controller::TrajectoryState::DONE);
    // Sanity: it did not crash (we are still running this assertion).
}

// ═════════════════════════════════════════════════════════════════════════════
// Test 2: Extreme θ values (0° → 180°) produce valid poses without NaN
// ═════════════════════════════════════════════════════════════════════════════

TEST_F(DoorTrajectoryEdgeTest, ExtremeThetaValuesProduceValidPoses) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("idle_delay_sec", 0.0);
    opts.append_parameter_override("tick_rate", 100.0);
    opts.append_parameter_override("theta_max_deg", 180.0);
    opts.append_parameter_override("theta_step_deg", 30.0);
    opts.append_parameter_override("phi_values_deg", std::vector<double>({0.0, 45.0, 90.0}));
    auto node = std::make_shared<EdgeTestNode>(opts);

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);  // all plans succeed
    node->setIdleStartTimeForTesting(node->get_clock()->now() -
                                     rclcpp::Duration::from_seconds(1.0));

    // Navigate through to PREPARE_WAYPOINTS → waypoints computed.
    spinUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);

    const auto& waypoints = node->getWaypoints();
    EXPECT_GT(waypoints.size(), 0u) << "Waypoints must be non-empty";

    // Expected: theta ∈ {0, 30, 60, 90, 120, 150, 180} = 7 steps × 3 phi = 21
    int expected_theta_steps = (180 / 30) + 1;
    int expected_phi_count = 3;
    size_t expected_waypoints = static_cast<size_t>(expected_theta_steps) * expected_phi_count;
    EXPECT_EQ(waypoints.size(), expected_waypoints)
        << expected_theta_steps << " θ steps × " << expected_phi_count << " φ values";

    // Verify no waypoint pose contains NaN.
    for (size_t i = 0; i < waypoints.size(); ++i) {
        EXPECT_FALSE(poseHasNaN(waypoints[i])) << "Waypoint " << i << " has NaN component";
    }

    // Verify quaternions are normalised (|q| ≈ 1.0).
    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& o = waypoints[i].orientation;
        double norm_sq = o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w;
        EXPECT_NEAR(norm_sq, 1.0, 1e-6) << "Waypoint " << i << " quaternion not normalised";
    }

    // Let the full sequence complete.
    node->setPlanResult(true);
    spinUntilState(node.get(), omr_controller::TrajectoryState::DONE, std::chrono::seconds(5));
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::DONE);
}

// ═════════════════════════════════════════════════════════════════════════════
// Test 3: Approach from non-home with collision objects set up before approach
// ═════════════════════════════════════════════════════════════════════════════

TEST_F(DoorTrajectoryEdgeTest, ApproachFromNonHomeSetsUpCollisionObjects) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("idle_delay_sec", 0.0);
    opts.append_parameter_override("tick_rate", 100.0);
    auto node = std::make_shared<EdgeTestNode>(opts);

    // Arm is NOT at home — simulates a pose that might intersect the door
    // during the approach planning phase.
    std::vector<double> non_home = {0.5, 0.3, -0.2, 0.1, 0.4, -0.3};
    node->setJointPositionsForTesting(non_home);
    node->setJointHomeResult(true);  // joint-space home approach succeeds
    node->setPlanResult(true);       // Cartesian approach plan succeeds
    node->setIdleStartTimeForTesting(node->get_clock()->now() -
                                     rclcpp::Duration::from_seconds(1.0));

    // Verify state transitions: APPROACH_HOME → PLAN_APPROACH → EXECUTE_APPROACH
    spinUntilState(node.get(), omr_controller::TrajectoryState::PLAN_APPROACH);
    EXPECT_TRUE(node->jointHomeCalled())
        << "Joint home move must be called when arm is not at home";
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLAN_APPROACH);

    // Let the approach plan+execute complete.
    spinUntilState(node.get(), omr_controller::TrajectoryState::EXECUTE_APPROACH);
    EXPECT_TRUE(node->setupDoorCollisionCalled())
        << "Collision objects must be added before approach execution";
    EXPECT_TRUE(node->door_objects_added_);
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::EXECUTE_APPROACH);

    // Continue to verify sequence begins (PREPARE_WAYPOINTS → PLANNING_WAYPOINT).
    spinUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);
}

// ═════════════════════════════════════════════════════════════════════════════
// Test 4: Missing T_armBase_doorHinge param — node starts normally with default
// ═════════════════════════════════════════════════════════════════════════════

TEST_F(DoorTrajectoryEdgeTest, MissingTransformParamDefaultsToZero) {
    // Construct with NO explicit T_armBase_doorHinge override — uses default.
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("idle_delay_sec", 0.1);
    opts.append_parameter_override("tick_rate", 100.0);
    auto node = std::make_shared<EdgeTestNode>(opts);

    // Default is [0, 0, 0, 0, 0, 0].
    const auto& T = node->T_armBase_doorHinge();
    ASSERT_EQ(T.size(), 6u);
    for (double val : T) {
        EXPECT_DOUBLE_EQ(val, 0.0);
    }

    // The node should start and tick without crash.
    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(node->get_clock()->now() -
                                     rclcpp::Duration::from_seconds(1.0));

    spinUntilState(node.get(), omr_controller::TrajectoryState::PLAN_APPROACH);
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLAN_APPROACH);

    // Verify that the node did not crash during waypoint preparation
    // (which uses T_armBase_doorHinge_ in computePoseForThetaPhi).
    spinUntilState(node.get(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::PLANNING_WAYPOINT);

    const auto& waypoints = node->getWaypoints();
    EXPECT_GT(waypoints.size(), 0u) << "Waypoints should be generated even with default transform";

    // Verify waypoints are valid (no NaN) despite default zero transform.
    for (size_t i = 0; i < waypoints.size(); ++i) {
        EXPECT_FALSE(poseHasNaN(waypoints[i]))
            << "Waypoint " << i << " has NaN with default transform";
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Test 5: θ locked per plan cycle — poses match pre-computed waypoint thetas
// ═════════════════════════════════════════════════════════════════════════════

TEST_F(DoorTrajectoryEdgeTest, ThetaLockedPerPlanCycle) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("idle_delay_sec", 0.0);
    opts.append_parameter_override("tick_rate", 100.0);
    opts.append_parameter_override("theta_max_deg", 20.0);
    opts.append_parameter_override("theta_step_deg", 10.0);
    opts.append_parameter_override("phi_values_deg", std::vector<double>({0.0, 90.0}));
    auto node = std::make_shared<EdgeTestNode>(opts);

    node->setJointPositionsForTesting(node->home_joints());
    node->setPlanResult(true);
    node->setIdleStartTimeForTesting(node->get_clock()->now() -
                                     rclcpp::Duration::from_seconds(1.0));

    // Navigate through to full completion.
    spinUntilState(node.get(), omr_controller::TrajectoryState::DONE, std::chrono::seconds(5));
    EXPECT_EQ(node->getCurrentState(), omr_controller::TrajectoryState::DONE);

    // Verify waypoint count: θ ∈ {0, 10, 20} = 3, φ ∈ {0, 90} = 2 → 6.
    const auto& waypoints = node->getWaypoints();
    EXPECT_EQ(waypoints.size(), 6u);
    EXPECT_GE(node->planCallCount(), 6) << "Expected at least 6 plan calls (one per waypoint)";

    // Verify that every plan call matches its corresponding pre-computed
    // waypoint.  This proves θ is locked at PREPARE_WAYPOINTS time and not
    // re-computed on every tick.
    const auto& planned = node->allPlannedPoses();
    ASSERT_GE(planned.size(), waypoints.size())
        << "Planned poses must include all waypoints (+ approach)";

    // The first entry in planned is the approach pose (θ=0, φ=0),
    // subsequent entries should match waypoints.
    size_t approach_offset = planned.size() - waypoints.size();
    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& wp = waypoints[i];
        const auto& pp = planned[approach_offset + i];

        EXPECT_NEAR(wp.position.x, pp.position.x, 1e-9)
            << "Waypoint " << i << " position.x mismatch";
        EXPECT_NEAR(wp.position.y, pp.position.y, 1e-9)
            << "Waypoint " << i << " position.y mismatch";
        EXPECT_NEAR(wp.position.z, pp.position.z, 1e-9)
            << "Waypoint " << i << " position.z mismatch";
        EXPECT_NEAR(wp.orientation.x, pp.orientation.x, 1e-9)
            << "Waypoint " << i << " orientation.x mismatch";
        EXPECT_NEAR(wp.orientation.y, pp.orientation.y, 1e-9)
            << "Waypoint " << i << " orientation.y mismatch";
        EXPECT_NEAR(wp.orientation.z, pp.orientation.z, 1e-9)
            << "Waypoint " << i << " orientation.z mismatch";
        EXPECT_NEAR(wp.orientation.w, pp.orientation.w, 1e-9)
            << "Waypoint " << i << " orientation.w mismatch";
    }

    // Verify that different (θ, φ) pairs produce different poses.
    // With identity T_armBase_doorHinge, the position depends primarily on
    // φ (not θ), and the orientation depends on both θ and φ.
    // Compare waypoint[0] (θ=0, φ=0) with waypoint[1] (θ=0, φ=90):
    // these have the same θ but different φ → positions MUST differ.
    bool poses_differ = false;
    if (waypoints.size() >= 2) {
        // Position: φ changes the reach, so waypoints with different φ
        // must have demonstrably different positions.
        double dx = std::abs(waypoints[0].position.x - waypoints[1].position.x);
        double dy = std::abs(waypoints[0].position.y - waypoints[1].position.y);
        double dz = std::abs(waypoints[0].position.z - waypoints[1].position.z);
        double dqw = std::abs(waypoints[0].orientation.w - waypoints[1].orientation.w);
        poses_differ = (dx > 1e-6 || dy > 1e-6 || dz > 1e-6 || dqw > 1e-6);
    }
    EXPECT_TRUE(poses_differ) << "Different (θ, φ) pairs must produce different waypoint poses "
                              << "(θ-locking mechanism: θ and φ are baked into pre-computed poses)";
}
