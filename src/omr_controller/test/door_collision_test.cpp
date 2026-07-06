#include <cmath>
#include <gtest/gtest.h>

#include <memory>

#include "omr_controller/door_trajectory_node.hpp"

class DoorCollisionTest : public omr_controller::DoorTrajectoryNode {
public:
    explicit DoorCollisionTest(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : DoorTrajectoryNode(options) {}

    using DoorTrajectoryNode::buildDoorFrameMsg;
    using DoorTrajectoryNode::buildDoorPanelMsg;
};

class DoorCollisionObjectTest : public ::testing::Test {
protected:
    void SetUp() override { node_ = std::make_shared<DoorCollisionTest>(); }

    std::shared_ptr<DoorCollisionTest> node_;
};

// ──── Door frame message structure ─────────────────────────────────────────

TEST_F(DoorCollisionObjectTest, BuildDoorFrameMsgHasCorrectStructure) {
    auto msg = node_->buildDoorFrameMsg();

    EXPECT_EQ(msg.id, "door_frame");
    EXPECT_EQ(msg.header.frame_id, node_->get_parameter("planning_frame").get_value<std::string>());
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
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].position.z, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.w, 1.0);
}

// ──── Door panel message structure at theta = 0 ────────────────────────────

TEST_F(DoorCollisionObjectTest, BuildDoorPanelMsgAtThetaZero) {
    auto msg = node_->buildDoorPanelMsg(0.0);

    EXPECT_EQ(msg.id, "door_panel");
    EXPECT_EQ(msg.operation, moveit_msgs::msg::CollisionObject::ADD);

    ASSERT_EQ(msg.primitives.size(), 1u);
    EXPECT_EQ(msg.primitives[0].type, shape_msgs::msg::SolidPrimitive::BOX);

    ASSERT_EQ(msg.primitives[0].dimensions.size(), 3u);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X], 2.0);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y], 0.05);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z], 0.8);

    ASSERT_EQ(msg.primitive_poses.size(), 1u);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].position.x, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].position.y, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].position.z, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(msg.primitive_poses[0].orientation.w, 1.0);
}

// ──── Quaternion correctness for known angles ──────────────────────────────

TEST_F(DoorCollisionObjectTest, QuaternionForTheta30Deg) {
    double theta = 30.0 * M_PI / 180.0;
    auto msg = node_->buildDoorPanelMsg(theta);

    const auto& q = msg.primitive_poses[0].orientation;
    double half = 15.0 * M_PI / 180.0;
    EXPECT_NEAR(q.z, std::sin(half), 1e-9);
    EXPECT_NEAR(q.w, std::cos(half), 1e-9);
    EXPECT_DOUBLE_EQ(q.x, 0.0);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
}

TEST_F(DoorCollisionObjectTest, QuaternionForTheta90Deg) {
    double theta = 90.0 * M_PI / 180.0;
    auto msg = node_->buildDoorPanelMsg(theta);

    const auto& q = msg.primitive_poses[0].orientation;
    double half = 45.0 * M_PI / 180.0;
    EXPECT_NEAR(q.z, std::sin(half), 1e-9);
    EXPECT_NEAR(q.w, std::cos(half), 1e-9);
    EXPECT_DOUBLE_EQ(q.x, 0.0);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
}

TEST_F(DoorCollisionObjectTest, QuaternionForTheta360Deg) {
    double theta = 360.0 * M_PI / 180.0;
    auto msg = node_->buildDoorPanelMsg(theta);

    const auto& q = msg.primitive_poses[0].orientation;
    // Full rotation — identity up to quaternion double-cover sign
    EXPECT_NEAR(std::abs(q.w), 1.0, 1e-9);
    EXPECT_NEAR(q.x, 0.0, 1e-9);
    EXPECT_NEAR(q.y, 0.0, 1e-9);
    EXPECT_NEAR(q.z, 0.0, 1e-9);
}

// ──── Different orientations produce different messages ────────────────────

TEST_F(DoorCollisionObjectTest, ThetaZeroVsTheta90ProducesDifferentOrientation) {
    auto msg0 = node_->buildDoorPanelMsg(0.0);
    auto msg90 = node_->buildDoorPanelMsg(90.0 * M_PI / 180.0);

    const auto& q0 = msg0.primitive_poses[0].orientation;
    const auto& q90 = msg90.primitive_poses[0].orientation;

    bool differs = (std::abs(q0.x - q90.x) > 1e-9) || (std::abs(q0.y - q90.y) > 1e-9) ||
                   (std::abs(q0.z - q90.z) > 1e-9) || (std::abs(q0.w - q90.w) > 1e-9);
    EXPECT_TRUE(differs);
}

// ──── Position unchanged across theta values ───────────────────────────────

TEST_F(DoorCollisionObjectTest, PanelPositionUnchangedWithTheta) {
    auto msg0 = node_->buildDoorPanelMsg(0.0);
    auto msg45 = node_->buildDoorPanelMsg(45.0 * M_PI / 180.0);
    auto msg90 = node_->buildDoorPanelMsg(90.0 * M_PI / 180.0);

    for (const auto* msg : {&msg0, &msg45, &msg90}) {
        EXPECT_DOUBLE_EQ(msg->primitive_poses[0].position.x, 0.0);
        EXPECT_DOUBLE_EQ(msg->primitive_poses[0].position.y, 0.0);
        EXPECT_DOUBLE_EQ(msg->primitive_poses[0].position.z, 0.0);
    }
}

// ──── Custom dimensions are respected ──────────────────────────────────────

TEST(DoorCollisionCustomParamTest, CustomPanelSizeAffectsMsg) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("door_panel_size", std::vector<double>({1.0, 0.02, 0.5}));

    auto node = std::make_shared<DoorCollisionTest>(opts);
    auto msg = node->buildDoorPanelMsg(0.0);

    ASSERT_EQ(msg.primitives[0].dimensions.size(), 3u);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X], 1.0);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y], 0.02);
    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z], 0.5);
}

TEST(DoorCollisionCustomParamTest, CustomFrameRadiusAffectsMsg) {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("door_frame_radius", 0.1);

    auto node = std::make_shared<DoorCollisionTest>(opts);
    auto msg = node->buildDoorFrameMsg();

    EXPECT_DOUBLE_EQ(msg.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS],
                     0.1);
}
