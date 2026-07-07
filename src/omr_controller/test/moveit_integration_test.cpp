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

class DoorTrajectoryNodeMoveitTest : public omr_controller::DoorTrajectoryAction {
public:
    DoorTrajectoryNodeMoveitTest(const std::string& name, const BT::NodeConfig& config)
        : DoorTrajectoryAction(name, config) {}

    using DoorTrajectoryAction::ensureMoveGroup;
    using DoorTrajectoryAction::getJointPositionsForTesting;
    using DoorTrajectoryAction::jointStateCallback;
    using DoorTrajectoryAction::planAndExecuteToPose;
    using DoorTrajectoryAction::setJointPositionsForTesting;
    using DoorTrajectoryAction::waitForCompletion;
};

class MoveitIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        ros_node_ = std::make_shared<rclcpp::Node>("moveit_test");
        auto cfg = make_config(ros_node_);
        node_ = std::make_shared<DoorTrajectoryNodeMoveitTest>("door_traj", cfg);
        node_->executeTick();  // onStart() initialises
    }

    rclcpp::Node::SharedPtr ros_node_;
    std::shared_ptr<DoorTrajectoryNodeMoveitTest> node_;
};

TEST_F(MoveitIntegrationTest, PlanAndExecuteToPoseReturnsFalseWhenPlanFails) {
    geometry_msgs::msg::Pose target;
    target.position.x = 0.5;
    target.position.y = 0.0;
    target.position.z = 0.3;
    target.orientation.w = 1.0;

    bool result = node_->planAndExecuteToPose(target);
    EXPECT_FALSE(result) << "planAndExecuteToPose should return false when "
                            "move_group is not available";
}

TEST_F(MoveitIntegrationTest, JointStateCallbackCachesPositions) {
    auto msg = std::make_shared<sensor_msgs::msg::JointState>();
    msg->position = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

    node_->jointStateCallback(msg);

    auto cached = node_->getJointPositionsForTesting();
    ASSERT_EQ(cached.size(), 6u);
    for (size_t i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(cached[i], msg->position[i]) << "i=" << i;
    }
}

TEST_F(MoveitIntegrationTest, WaitForCompletionReturnsTrueWhenJointsMatch) {
    std::vector<double> target{1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
    node_->setJointPositionsForTesting(target);

    bool result = node_->waitForCompletion(target, 1.0);
    EXPECT_TRUE(result) << "waitForCompletion should return true when all "
                           "joints are within tolerance";
}

TEST_F(MoveitIntegrationTest, WaitForCompletionReturnsFalseOnTimeout) {
    std::vector<double> target{1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
    std::vector<double> far_away{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    node_->setJointPositionsForTesting(far_away);

    bool result = node_->waitForCompletion(target, 0.1);
    EXPECT_FALSE(result) << "waitForCompletion should return false on timeout";
}

}  // namespace
