#include <gtest/gtest.h>

#include <memory>

#include "omr_controller/clients/arm_client.hpp"
#include "omr_controller/clients/base_client.hpp"
#include "omr_controller/clients/gripper_client.hpp"
#include "omr_controller/clients/motor_client.hpp"
#include "omr_controller/clients/vision_client.hpp"
#include "test_helpers.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace omr_controller;

// ---------------------------------------------------------------------------
// IntegrationTest — shared rclcpp lifecycle for all integration tests.
// ---------------------------------------------------------------------------
class IntegrationTest : public ::testing::Test {
protected:
    void SetUp() override { node_ = std::make_shared<rclcpp::Node>("integration_test"); }

    rclcpp::Node::SharedPtr node_;
};

// ---------------------------------------------------------------------------
// Orchestrator-like flow: chain ArmClient moveJoints → GripperClient open.
// ---------------------------------------------------------------------------
TEST_F(IntegrationTest, ArmGripperSequence) {
    // Start mock FollowJointTrajectory action server (default arm action).
    auto arm_server = omr_controller::test::create_mock_jtc_server(
        node_.get(), "/arm_cm/follow_joint_trajectory");

    // Start mock GripperCommand action server (default gripper action).
    using GripperAction = control_msgs::action::GripperCommand;
    auto gripper_server = rclcpp_action::create_server<GripperAction>(
        node_.get(), "/gripper/follow_joint_trajectory",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const GripperAction::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperAction>>&) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperAction>>& gh) {
            auto result = std::make_shared<GripperAction::Result>();
            gh->succeed(result);
        });

    // Let the action servers register in the ROS graph.
    rclcpp::spin_some(node_);

    ArmClient arm(node_);
    GripperClient gripper(node_);

    // Step 1: send a joint trajectory goal.
    JointGoal goal;
    goal.positions = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    EXPECT_NO_THROW(arm.moveJoints(goal));

    // Step 2: open the gripper.
    EXPECT_TRUE(gripper.open());
}

// ---------------------------------------------------------------------------
// All five client types construct and can be used without conflicts.
// ---------------------------------------------------------------------------
TEST_F(IntegrationTest, AllFiveClientsConstructWithoutConflict) {
    // Stubs need no ROS infrastructure.
    MotorClientStub motor;
    BaseClientImpl base(node_);

    // VisionClient needs a fake camera.
    struct LocalFakeCamera : public ICamera {
        explicit LocalFakeCamera(cv::Mat img) : image_(std::move(img)) {}
        std::optional<cv::Mat> next() override {
            if (returned_) return std::nullopt;
            returned_ = true;
            return image_.clone();
        }
        omr_vision::camera::CameraIntrinsics depth_intrinsics() const override { return {}; }
        cv::Mat image_;
        bool returned_{false};
    };

    cv::Mat dummy(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
    VisionClient vision(std::make_unique<LocalFakeCamera>(dummy));

    // ArmClient and GripperClient need a dedicated node.
    auto other_node = std::make_shared<rclcpp::Node>("client_node");
    ArmClient arm(other_node);
    GripperClient gripper(other_node);

    // All five are alive; call a method on each to verify.
    EXPECT_NO_THROW({
        motor.enable();
        base.stop();
        vision.detect(dummy);
        arm.currentJoints();
        gripper.stop();
    });
}

// ---------------------------------------------------------------------------
// MotorClientStub and BaseClientImpl return expected defaults together.
// ---------------------------------------------------------------------------
TEST_F(IntegrationTest, MotorAndBaseClientsReturnDefaults) {
    MotorClientStub motor;
    BaseClientImpl base(node_);

    // Motor default values.
    EXPECT_FALSE(motor.enable());
    EXPECT_FALSE(motor.disable());
    EXPECT_FALSE(motor.isEnabled());
    EXPECT_FALSE(motor.setVelocity(1.5));
    EXPECT_FALSE(motor.stop());

    MotorState state = motor.getState();
    EXPECT_DOUBLE_EQ(state.position_rad, 0.0);
    EXPECT_DOUBLE_EQ(state.velocity_rad_s, 0.0);
    EXPECT_FALSE(state.connected);
    EXPECT_FALSE(state.servo_enabled);

    // Base default values — real impl returns true on move/stop.
    EXPECT_TRUE(base.move(0.3, 0.1));
    EXPECT_TRUE(base.stop());

    auto pose = base.getPose();
    ASSERT_EQ(pose.size(), 3u);
    EXPECT_DOUBLE_EQ(pose[0], 0.0);
    EXPECT_DOUBLE_EQ(pose[1], 0.0);
    EXPECT_DOUBLE_EQ(pose[2], 0.0);
}
