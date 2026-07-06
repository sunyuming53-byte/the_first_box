#include "omr_controller/state_machine/bt_factory.hpp"
#include "omr_controller/clients/arm_client.hpp"
#include "omr_controller/clients/gripper_client.hpp"
#include "omr_controller/clients/vision_client.hpp"
#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <stdexcept>
#include <string>

namespace {

// FakeCamera for VisionClient tests (no hardware required).
class FakeCamera : public omr_controller::ICamera {
public:
    explicit FakeCamera(cv::Mat image) : image_(std::move(image)) {}

    std::optional<cv::Mat> next() override {
        if (returned_) return std::nullopt;
        returned_ = true;
        return image_.clone();
    }

    omr_vision::camera::CameraIntrinsics depth_intrinsics() const override {
        omr_vision::camera::CameraIntrinsics intr;
        intr.K = (cv::Mat_<double>(3, 3) << 500.0, 0.0, 100.0,
                                              0.0, 500.0, 100.0,
                                              0.0, 0.0, 1.0);
        intr.dist_coeff = cv::Mat::zeros(1, 5, CV_64F);
        return intr;
    }

private:
    cv::Mat image_;
    bool returned_{false};
};

cv::Mat makeRedBlobImage(int width, int height) {
    cv::Mat img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::circle(img, cv::Point(width / 2, height / 2), 20,
               cv::Scalar(0, 0, 255), cv::FILLED);
    return img;
}

// -------------------------------------------------------------------------
// Helper: create a minimal BT::NodeConfig with a fresh blackboard.
// -------------------------------------------------------------------------
BT::NodeConfig make_config() {
    BT::NodeConfig cfg;
    cfg.blackboard = BT::Blackboard::create();
    return cfg;
}

}  // namespace

// ============================================================================
// BtRosTest — fixture with rclcpp::Node for tests that need real clients.
// ============================================================================
class BtRosTest : public ::testing::Test {
protected:
    void SetUp() override {
        node_ = std::make_shared<rclcpp::Node>("bt_test");
    }

    void TearDown() override {
        node_.reset();
    }

    rclcpp::Node::SharedPtr node_;
};

// ============================================================================
// MoveArmAction — uses real ArmClient with mock JTC action server.
// ============================================================================

TEST_F(BtRosTest, MoveArmActionParsesJointPositionsAndSucceeds) {
    auto server = omr_controller::test::create_mock_jtc_server(
        node_.get(), "/arm_cm/follow_joint_trajectory");
    rclcpp::spin_some(node_);

    omr_controller::ArmClient arm(node_);
    auto cfg = make_config();
    cfg.blackboard->set("arm_client", &arm);
    cfg.input_ports["joint_positions"] = "1.0,0.5,-0.3";
    cfg.input_ports["speed_ratio"] = "75.0";

    omr_controller::MoveArmAction node("move_arm", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(BtRosTest, MoveArmActionMissingArmClientThrows) {
    auto cfg = make_config();
    cfg.input_ports["joint_positions"] = "1.0,0.5";
    cfg.input_ports["speed_ratio"] = "50.0";

    omr_controller::MoveArmAction node("move_arm", cfg);
    // BT blackboard throws when a required key is missing
    EXPECT_THROW(node.executeTick(), std::exception);
}

TEST_F(BtRosTest, MoveArmActionDefaultSpeedRatioSucceeds) {
    auto server = omr_controller::test::create_mock_jtc_server(
        node_.get(), "/arm_cm/follow_joint_trajectory");
    rclcpp::spin_some(node_);

    omr_controller::ArmClient arm(node_);
    auto cfg = make_config();
    cfg.blackboard->set("arm_client", &arm);
    cfg.input_ports["joint_positions"] = "0.1,0.2";

    omr_controller::MoveArmAction node("move_arm", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(BtRosTest, MoveArmActionNoActionServerStillSucceeds) {
    // moveJoints is fire-and-forget — it returns void even if no server.
    omr_controller::ArmClient arm(node_);
    auto cfg = make_config();
    cfg.blackboard->set("arm_client", &arm);
    cfg.input_ports["joint_positions"] = "0.1,0.2";

    omr_controller::MoveArmAction node("move_arm", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

// ============================================================================
// GripperAction — uses real GripperClient with mock GripperCommand server.
// ============================================================================

TEST_F(BtRosTest, GripperActionOpenSucceeds) {
    auto server = omr_controller::test::create_mock_gripper_server(
        node_.get(), "/gripper/follow_joint_trajectory");
    rclcpp::spin_some(node_);

    omr_controller::GripperClient gripper(node_);
    auto cfg = make_config();
    cfg.blackboard->set("gripper_client", &gripper);
    cfg.input_ports["action"] = "open";
    cfg.input_ports["force"] = "80.0";

    omr_controller::GripperAction node("gripper", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(BtRosTest, GripperActionCloseSucceeds) {
    auto server = omr_controller::test::create_mock_gripper_server(
        node_.get(), "/gripper/follow_joint_trajectory");
    rclcpp::spin_some(node_);

    omr_controller::GripperClient gripper(node_);
    auto cfg = make_config();
    cfg.blackboard->set("gripper_client", &gripper);
    cfg.input_ports["action"] = "close";
    cfg.input_ports["force"] = "30.0";

    omr_controller::GripperAction node("gripper", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(BtRosTest, GripperActionInvalidActionReturnsFailure) {
    omr_controller::GripperClient gripper(node_);
    auto cfg = make_config();
    cfg.blackboard->set("gripper_client", &gripper);
    cfg.input_ports["action"] = "twist";

    omr_controller::GripperAction node("gripper", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(BtRosTest, GripperActionMissingClientThrows) {
    auto cfg = make_config();
    cfg.input_ports["action"] = "open";

    omr_controller::GripperAction node("gripper", cfg);
    // BT blackboard throws when a required key is missing
    EXPECT_THROW(node.executeTick(), std::exception);
}

// ============================================================================
// DetectObjectAction — uses real VisionClient with FakeCamera.
// ============================================================================

TEST_F(BtRosTest, DetectObjectActionWithRedBlobReturnsSuccess) {
    cv::Mat img = makeRedBlobImage(200, 200);
    omr_controller::VisionClient vision(std::make_unique<FakeCamera>(img));

    auto cfg = make_config();
    cfg.blackboard->set("vision_client", &vision);

    omr_controller::DetectObjectAction node("detect", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

    int count = 0;
    bool found = cfg.blackboard->get("detection_count", count);
    EXPECT_TRUE(found);
    EXPECT_EQ(count, 1);
}

TEST_F(BtRosTest, DetectObjectActionEmptyCameraReturnsFailure) {
    cv::Mat black(200, 200, CV_8UC3, cv::Scalar(0, 0, 0));
    omr_controller::VisionClient vision(std::make_unique<FakeCamera>(black));

    auto cfg = make_config();
    cfg.blackboard->set("vision_client", &vision);

    omr_controller::DetectObjectAction node("detect", cfg);
    auto status = node.executeTick();

    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(BtRosTest, DetectObjectActionMissingClientThrows) {
    auto cfg = make_config();

    omr_controller::DetectObjectAction node("detect", cfg);
    // BT blackboard throws when a required key is missing
    EXPECT_THROW(node.executeTick(), std::exception);
}

// ============================================================================
// WaitAction — no ROS dependency, keeps original non-fixture style.
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
// BehaviorTree factory — integration tests (no ROS needed for tree creation).
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

    EXPECT_ANY_THROW(factory.createTreeFromText(xml));
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
// BtTreeLifecycle — tree-level integration tests with real clients.
// ============================================================================

TEST_F(BtRosTest, TicksThroughSequence) {
    omr_controller::ArmClient arm(node_);
    omr_controller::GripperClient gripper(node_);
    cv::Mat dummy(100, 100, CV_8UC3, cv::Scalar(0, 0, 255));
    omr_controller::VisionClient vision(std::make_unique<FakeCamera>(dummy));

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
    tree.rootBlackboard()->set("arm_client", &arm);
    tree.rootBlackboard()->set("gripper_client", &gripper);
    tree.rootBlackboard()->set("vision_client", &vision);

    ASSERT_EQ(tree.subtrees[0]->nodes.size(), 3u);

    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS)
            << "tick " << i << " should succeed";
    }
}

TEST_F(BtRosTest, FallbackOnDetectFailure) {
    omr_controller::ArmClient arm(node_);
    omr_controller::GripperClient gripper(node_);
    cv::Mat black(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
    omr_controller::VisionClient vision(std::make_unique<FakeCamera>(black));

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
    tree.rootBlackboard()->set("arm_client", &arm);
    tree.rootBlackboard()->set("gripper_client", &gripper);
    tree.rootBlackboard()->set("vision_client", &vision);

    ASSERT_EQ(tree.subtrees[0]->nodes.size(), 3u);

    BT::NodeStatus status = tree.tickOnce();
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

    int count = -1;
    bool has_count = tree.rootBlackboard()->get("detection_count", count);
    EXPECT_FALSE(has_count);
}
