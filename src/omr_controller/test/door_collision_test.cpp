#include <cmath>
#include <gtest/gtest.h>

#include <memory>

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

// Test adapter exposing collision object builders.
class DoorCollisionTest : public omr_controller::DoorTrajectoryAction {
public:
    DoorCollisionTest(const std::string& name, const BT::NodeConfig& config)
        : DoorTrajectoryAction(name, config) {}

    using DoorTrajectoryAction::buildDoorFrameMsg;
    using DoorTrajectoryAction::buildDoorPanelMsg;
};

class DoorCollisionObjectTest : public ::testing::Test {
protected:
    void SetUp() override {
        ros_node_ = std::make_shared<rclcpp::Node>("collision_test");
        auto cfg = make_config(ros_node_);
        node_ = std::make_shared<DoorCollisionTest>("door_traj", cfg);
        node_->executeTick();  // onStart() initialises ports
    }

    rclcpp::Node::SharedPtr ros_node_;
    std::shared_ptr<DoorCollisionTest> node_;
};

const cv::Mat kIdentity = cv::Mat::eye(4, 4, CV_64F);

// ──── Door frame message structure ─────────────────────────────────────────

TEST_F(DoorCollisionObjectTest, BuildDoorFrameMsgHasCorrectStructure) {
    auto msg = node_->buildDoorFrameMsg(kIdentity);

    EXPECT_EQ(msg.id, "door_frame");
    EXPECT_EQ(msg.header.frame_id, "base_link");
    EXPECT_EQ(msg.operation, moveit_msgs::msg::CollisionObject::ADD);

    ASSERT_EQ(msg.primitives.size(), 1u);
    EXPECT_EQ(msg.primitives[0].type, shape_msgs::msg::SolidPrimitive::CYLINDER);

    ASSERT_EQ(msg.primitives[0].dimensions.size(), 2u);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT],
                     0.8);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS],
                     0.05);

    ASSERT_EQ(msg.primitive_poses.size(), 1u);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].position.x, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].position.y, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].position.z, 0.4);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.w, 1.0);
}

// ──── Door panel message structure at theta = 0 ────────────────────────────

TEST_F(DoorCollisionObjectTest, BuildDoorPanelMsgAtThetaZero) {
    auto msg = node_->buildDoorPanelMsg(0.0, kIdentity);

    EXPECT_EQ(msg.id, "door_panel");
    EXPECT_EQ(msg.operation, moveit_msgs::msg::CollisionObject::ADD);

    ASSERT_EQ(msg.primitives.size(), 1u);
    EXPECT_EQ(msg.primitives[0].type, shape_msgs::msg::SolidPrimitive::BOX);

    ASSERT_EQ(msg.primitives[0].dimensions.size(), 3u);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X], 2.0);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y], 0.05);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z], 0.8);

    ASSERT_EQ(msg.primitive_poses.size(), 1u);
    // hinge-frame pose at (width/2, 0, height/2) = (1.0, 0, 0.4) transformed by identity
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].position.x, 1.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].position.y, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].position.z, 0.4);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.w, 1.0);
}

// ──── Quaternion correctness for known angles ──────────────────────────────

TEST_F(DoorCollisionObjectTest, QuaternionForTheta30Deg) {
    double theta = 30.0 * M_PI / 180.0;
    auto msg = node_->buildDoorPanelMsg(theta, kIdentity);

    const auto& q = msg.primitive_poses[0].orientation;
    double half = 15.0 * M_PI / 180.0;
    EXPECT_NEAR(q.z, std::sin(half), 1e-9);
    EXPECT_NEAR(q.w, std::cos(half), 1e-9);
    EXPECT_DOUBLE_EQ(q.x, 0.0);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
}

TEST_F(DoorCollisionObjectTest, QuaternionForTheta90Deg) {
    double theta = 90.0 * M_PI / 180.0;
    auto msg = node_->buildDoorPanelMsg(theta, kIdentity);

    const auto& q = msg.primitive_poses[0].orientation;
    double half = 45.0 * M_PI / 180.0;
    EXPECT_NEAR(q.z, std::sin(half), 1e-9);
    EXPECT_NEAR(q.w, std::cos(half), 1e-9);
    EXPECT_DOUBLE_EQ(q.x, 0.0);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
}

TEST_F(DoorCollisionObjectTest, QuaternionForTheta360Deg) {
    double theta = 360.0 * M_PI / 180.0;
    auto msg = node_->buildDoorPanelMsg(theta, kIdentity);

    const auto& q = msg.primitive_poses[0].orientation;
    EXPECT_NEAR(std::abs(q.w), 1.0, 1e-9);
    EXPECT_NEAR(q.x, 0.0, 1e-9);
    EXPECT_NEAR(q.y, 0.0, 1e-9);
    EXPECT_NEAR(q.z, 0.0, 1e-9);
}

// ──── Different orientations produce different messages ────────────────────

TEST_F(DoorCollisionObjectTest, ThetaZeroVsTheta90ProducesDifferentOrientation) {
    auto msg0 = node_->buildDoorPanelMsg(0.0, kIdentity);
    auto msg90 = node_->buildDoorPanelMsg(90.0 * M_PI / 180.0, kIdentity);

    const auto& q0 = msg0.primitive_poses[0].orientation;
    const auto& q90 = msg90.primitive_poses[0].orientation;

    bool differs = (std::abs(q0.x - q90.x) > 1e-9) || (std::abs(q0.y - q90.y) > 1e-9) ||
                   (std::abs(q0.z - q90.z) > 1e-9) || (std::abs(q0.w - q90.w) > 1e-9);
    EXPECT_TRUE(differs);
}

// ──── Position varies with theta (panel center rotates around hinge) ───────

TEST_F(DoorCollisionObjectTest, PanelPositionVariesWithTheta) {
    auto msg0 = node_->buildDoorPanelMsg(0.0, kIdentity);
    auto msg45 = node_->buildDoorPanelMsg(45.0 * M_PI / 180.0, kIdentity);
    auto msg90 = node_->buildDoorPanelMsg(90.0 * M_PI / 180.0, kIdentity);

    // theta=0: panel at (1.0, 0, 0.4)
    EXPECT_NEAR(msg0.primitive_poses[0].position.x, 1.0, 1e-9);
    EXPECT_NEAR(msg0.primitive_poses[0].position.y, 0.0, 1e-9);
    EXPECT_NEAR(msg0.primitive_poses[0].position.z, 0.4, 1e-9);

    // theta=45: panel at (width/2*cos45, width/2*sin45, height/2) = (0.707, 0.707, 0.4)
    double s45 = std::sin(45.0 * M_PI / 180.0);
    EXPECT_NEAR(msg45.primitive_poses[0].position.x, s45, 1e-9);
    EXPECT_NEAR(msg45.primitive_poses[0].position.y, s45, 1e-9);
    EXPECT_NEAR(msg45.primitive_poses[0].position.z, 0.4, 1e-9);

    // theta=90: panel at (0, 1.0, 0.4)
    EXPECT_NEAR(msg90.primitive_poses[0].position.x, 0.0, 1e-9);
    EXPECT_NEAR(msg90.primitive_poses[0].position.y, 1.0, 1e-9);
    EXPECT_NEAR(msg90.primitive_poses[0].position.z, 0.4, 1e-9);
}

// ──── Custom dimensions are respected ──────────────────────────────────────

TEST(DoorCollisionCustomParamTest, CustomPanelSizeAffectsMsg) {
    auto ros_node = std::make_shared<rclcpp::Node>("custom_collision_test");
    auto cfg = make_config(ros_node);
    cfg.input_ports["door_panel_size"] = "1.0,0.02,0.5";

    auto node = std::make_shared<DoorCollisionTest>("door_traj", cfg);
    node->executeTick();

    auto msg = node->buildDoorPanelMsg(0.0, kIdentity);
    ASSERT_EQ(msg.primitives[0].dimensions.size(), 3u);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X], 1.0);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y], 0.02);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z], 0.5);
}

TEST(DoorCollisionCustomParamTest, CustomFrameRadiusAffectsMsg) {
    auto ros_node = std::make_shared<rclcpp::Node>("custom_collision_test");
    auto cfg = make_config(ros_node);
    cfg.input_ports["door_frame_radius"] = "0.1";

    auto node = std::make_shared<DoorCollisionTest>("door_traj", cfg);
    node->executeTick();

    auto msg = node->buildDoorFrameMsg(kIdentity);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS],
                     0.1);
}

}  // namespace
