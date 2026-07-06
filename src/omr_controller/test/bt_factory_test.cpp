#include <omr_controller/state_machine/bt_factory.hpp>

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>

#include <stdexcept>
#include <string>

// ============================================================================
// Minimal mock client structs — simple flags for verifying method calls.
// These are NOT the real ROS2 clients and do not require rclcpp::Node.
// ============================================================================

struct MockArmClient {
    bool move_joints_called{false};
    std::string last_positions;
    double last_speed_ratio{0.0};

    void moveJoints(const omr_controller::JointGoal& goal) {
        move_joints_called = true;
        last_speed_ratio = goal.speed_ratio;
    }
};

struct MockGripperClient {
    bool open_called{false};
    bool close_called{false};
    double last_force{0.0};

    bool open(double force_pct = 50) {
        open_called = true;
        last_force = force_pct;
        return true;
    }

    bool close(double force_pct = 50) {
        close_called = true;
        last_force = force_pct;
        return true;
    }
};

struct MockVisionClient {
    bool next_detection_called{false};
    std::vector<omr_controller::DetectionResult> canned_results;

    std::optional<std::vector<omr_controller::DetectionResult>> next_detection() {
        next_detection_called = true;
        if (canned_results.empty()) {
            return std::nullopt;
        }
        return canned_results;
    }
};

// ============================================================================
// Helper: create a minimal BT::NodeConfig with a fresh blackboard.
// ============================================================================
static BT::NodeConfig make_config() {
    BT::NodeConfig cfg;
    cfg.blackboard = BT::Blackboard::create();
    return cfg;
}

// ============================================================================
// MoveArmAction — direct node tests
// ============================================================================

TEST(MoveArmActionTest, ParsesJointPositionsAndCallsArm) {
    MockArmClient mock_arm;
    auto cfg = make_config();
    cfg.blackboard->set("arm_client", &mock_arm);
    cfg.input_ports["joint_positions"] = "1.0,0.5,-0.3";
    cfg.input_ports["speed_ratio"] = "75.0";

    omr_controller::MoveArmAction node("move_arm", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(mock_arm.move_joints_called);
    EXPECT_DOUBLE_EQ(mock_arm.last_speed_ratio, 75.0);
}

TEST(MoveArmActionTest, MissingArmClientReturnsFailure) {
    auto cfg = make_config();
    cfg.input_ports["joint_positions"] = "1.0,0.5";
    cfg.input_ports["speed_ratio"] = "50.0";

    omr_controller::MoveArmAction node("move_arm", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(MoveArmActionTest, DefaultSpeedRatio) {
    MockArmClient mock_arm;
    auto cfg = make_config();
    cfg.blackboard->set("arm_client", &mock_arm);
    cfg.input_ports["joint_positions"] = "0.1,0.2";

    omr_controller::MoveArmAction node("move_arm", cfg);
    node.executeTick();

    EXPECT_DOUBLE_EQ(mock_arm.last_speed_ratio, 50.0);
}

// ============================================================================
// GripperAction — direct node tests
// ============================================================================

TEST(GripperActionTest, OpenCallsGripperOpen) {
    MockGripperClient mock_gripper;
    auto cfg = make_config();
    cfg.blackboard->set("gripper_client", &mock_gripper);
    cfg.input_ports["action"] = "open";
    cfg.input_ports["force"] = "80.0";

    omr_controller::GripperAction node("gripper", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(mock_gripper.open_called);
    EXPECT_FALSE(mock_gripper.close_called);
    EXPECT_DOUBLE_EQ(mock_gripper.last_force, 80.0);
}

TEST(GripperActionTest, CloseCallsGripperClose) {
    MockGripperClient mock_gripper;
    auto cfg = make_config();
    cfg.blackboard->set("gripper_client", &mock_gripper);
    cfg.input_ports["action"] = "close";
    cfg.input_ports["force"] = "30.0";

    omr_controller::GripperAction node("gripper", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(mock_gripper.close_called);
    EXPECT_FALSE(mock_gripper.open_called);
    EXPECT_DOUBLE_EQ(mock_gripper.last_force, 30.0);
}

TEST(GripperActionTest, InvalidActionReturnsFailure) {
    MockGripperClient mock_gripper;
    auto cfg = make_config();
    cfg.blackboard->set("gripper_client", &mock_gripper);
    cfg.input_ports["action"] = "twist";

    omr_controller::GripperAction node("gripper", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

// ============================================================================
// DetectObjectAction — direct node tests
// ============================================================================

TEST(DetectObjectActionTest, HasDetectionsReturnsSuccess) {
    MockVisionClient mock_vision;
    mock_vision.canned_results = {
        omr_controller::DetectionResult{"obj1", {10, 20}, 0.95},
        omr_controller::DetectionResult{"obj2", {30, 40}, 0.80},
    };

    auto cfg = make_config();
    cfg.blackboard->set("vision_client", &mock_vision);

    omr_controller::DetectObjectAction node("detect", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(mock_vision.next_detection_called);

    int count = 0;
    cfg.blackboard->get("detection_count", count);
    EXPECT_EQ(count, 2);
}

TEST(DetectObjectActionTest, NoDetectionsReturnsFailure) {
    MockVisionClient mock_vision;  // canned_results empty by default

    auto cfg = make_config();
    cfg.blackboard->set("vision_client", &mock_vision);

    omr_controller::DetectObjectAction node("detect", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
    EXPECT_TRUE(mock_vision.next_detection_called);
}

// ============================================================================
// WaitAction — direct node tests
// ============================================================================

TEST(WaitActionTest, SleepsDurationAndReturnsSuccess) {
    auto cfg = make_config();
    cfg.input_ports["duration_ms"] = "10";

    omr_controller::WaitAction node("wait", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(WaitActionTest, ZeroDurationReturnsSuccess) {
    auto cfg = make_config();
    cfg.input_ports["duration_ms"] = "0";

    omr_controller::WaitAction node("wait", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

// ============================================================================
// BehaviorTree factory — integration tests
// ============================================================================

TEST(BehaviorTreeFactoryTest, BuildTreeFromXmlAndTick) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<omr_controller::WaitAction>("WaitAction");

    const std::string xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="TestTree">
    <Sequence>
      <WaitAction duration_ms="50"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

    auto tree = factory.createTreeFromText(xml);

    // Verify tree has 1 node under the root
    EXPECT_EQ(tree.subtrees.size(), 1u);
    EXPECT_EQ(tree.subtrees[0]->nodes.size(), 2u);  // Sequence + WaitAction

    auto status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(BehaviorTreeFactoryTest, UnregisteredNodeThrows) {
    BT::BehaviorTreeFactory factory;

    const std::string xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="TestTree">
    <UnknownNode/>
  </BehaviorTree>
</root>
)";

    EXPECT_THROW(factory.createTreeFromText(xml), std::runtime_error);
}

TEST(BehaviorTreeFactoryTest, RegisterAllNodesAndCreateTree) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<omr_controller::MoveArmAction>("MoveArmAction");
    factory.registerNodeType<omr_controller::GripperAction>("GripperAction");
    factory.registerNodeType<omr_controller::DetectObjectAction>("DetectObjectAction");
    factory.registerNodeType<omr_controller::WaitAction>("WaitAction");

    const std::string xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="TestTree">
    <Sequence>
      <DetectObjectAction/>
      <WaitAction duration_ms="10"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

    auto tree = factory.createTreeFromText(xml);
    EXPECT_EQ(tree.subtrees.size(), 1u);
    EXPECT_EQ(tree.subtrees[0]->nodes.size(), 3u);  // Sequence + 2 actions
}

// ============================================================================
// BtTreeLifecycle — tree-level integration tests with mock clients
// ============================================================================

TEST(BtTreeLifecycle, TicksThroughSequence) {
    MockArmClient mock_arm;
    MockGripperClient mock_gripper;
    MockVisionClient mock_vision;

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<omr_controller::MoveArmAction>("MoveArmAction");
    factory.registerNodeType<omr_controller::GripperAction>("GripperAction");
    factory.registerNodeType<omr_controller::DetectObjectAction>("DetectObjectAction");
    factory.registerNodeType<omr_controller::WaitAction>("WaitAction");

    const std::string xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="TestTree">
    <Sequence>
      <WaitAction duration_ms="10"/>
      <WaitAction duration_ms="10"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("arm_client", &mock_arm);
    tree.rootBlackboard()->set("gripper_client", &mock_gripper);
    tree.rootBlackboard()->set("vision_client", &mock_vision);

    // Tree structure: root Sequence + 2 × WaitAction
    ASSERT_EQ(tree.subtrees[0]->nodes.size(), 3u);

    // Each tick runs the full Sequence — both WaitActions sleep 10ms then
    // return SUCCESS. Three ticks means each WaitAction executed 3 times.
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS)
            << "tick " << i << " should succeed";
    }
}

TEST(BtTreeLifecycle, FallbackOnDetectFailure) {
    MockArmClient mock_arm;
    MockGripperClient mock_gripper;
    MockVisionClient mock_vision;  // empty canned_results → FAILURE

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<omr_controller::MoveArmAction>("MoveArmAction");
    factory.registerNodeType<omr_controller::GripperAction>("GripperAction");
    factory.registerNodeType<omr_controller::DetectObjectAction>("DetectObjectAction");
    factory.registerNodeType<omr_controller::WaitAction>("WaitAction");

    const std::string xml = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="TestTree">
    <Fallback>
      <DetectObjectAction/>
      <WaitAction duration_ms="10"/>
    </Fallback>
  </BehaviorTree>
</root>
)";

    auto tree = factory.createTreeFromText(xml);
    tree.rootBlackboard()->set("arm_client", &mock_arm);
    tree.rootBlackboard()->set("gripper_client", &mock_gripper);
    tree.rootBlackboard()->set("vision_client", &mock_vision);

    // Tree structure: root Fallback + 2 children
    ASSERT_EQ(tree.subtrees[0]->nodes.size(), 3u);

    BT::NodeStatus status = tree.tickOnce();

    // DetectObjectAction returns FAILURE (empty results). Fallback moves to
    // child 2 (WaitAction) which succeeds → Fallback returns SUCCESS.
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(mock_vision.next_detection_called);

    // Verify the detection count was NOT written (FAILURE branch)
    int count = -1;
    bool has_count = tree.rootBlackboard()->get("detection_count", count);
    EXPECT_FALSE(has_count);
}
