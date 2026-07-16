#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "omr_controller/clients/motor_client.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace omr_controller {

class MotorClientImplTestAccess {
public:
    static bool hasActiveGoal(const MotorClientImpl& client) {
        std::lock_guard<std::mutex> lock(client.mutex_);
        return client.activeGoal_ != nullptr;
    }
};

namespace {

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
using ServerGoalHandlePtr = std::shared_ptr<ServerGoalHandle>;

constexpr char kUnusedActionName[] = "/motor_client_impl_test/unused_action";
constexpr char kDefaultDaisActionName[] =
    "/dais_joint_trajectory_controller/follow_joint_trajectory";

class FakeTrajectoryServer {
public:
    FakeTrajectoryServer(const rclcpp::Node::SharedPtr& node, const std::string& actionName,
                         bool acceptGoals)
        : acceptGoals_(acceptGoals) {
        server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            node.get(), actionName,
            [this](const rclcpp_action::GoalUUID&,
                   std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    receivedGoals_.push_back(*goal);
                }
                condition_.notify_all();
                return acceptGoals_.load(std::memory_order_acquire)
                           ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                           : rclcpp_action::GoalResponse::REJECT;
            },
            [this](const ServerGoalHandlePtr&) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    ++cancelRequestCount_;
                }
                condition_.notify_all();
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const ServerGoalHandlePtr& goalHandle) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    activeGoals_.push_back(goalHandle);
                    ++acceptedGoalCount_;
                }
                condition_.notify_all();
            });
    }

    void setAcceptGoals(bool acceptGoals) {
        acceptGoals_.store(acceptGoals, std::memory_order_release);
    }

    bool waitForGoal(std::chrono::milliseconds timeout = 2s) {
        return waitForGoalCount(1, timeout);
    }

    bool waitForGoalCount(std::size_t count, std::chrono::milliseconds timeout = 2s) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, timeout,
                                   [this, count]() { return receivedGoals_.size() >= count; });
    }

    bool waitForAcceptedGoal(std::chrono::milliseconds timeout = 2s) {
        return waitForAcceptedGoalCount(1, timeout);
    }

    bool waitForAcceptedGoalCount(std::size_t count, std::chrono::milliseconds timeout = 2s) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, timeout,
                                   [this, count]() { return acceptedGoalCount_ >= count; });
    }

    bool waitForCancelCount(std::size_t count, std::chrono::milliseconds timeout = 2s) {
        std::unique_lock<std::mutex> lock(mutex_);
        return condition_.wait_for(lock, timeout,
                                   [this, count]() { return cancelRequestCount_ >= count; });
    }

    FollowJointTrajectory::Goal receivedGoal(std::size_t index = 0) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return receivedGoals_.at(index);
    }

    bool succeedActiveGoal() {
        ServerGoalHandlePtr goalHandle;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!activeGoals_.empty()) {
                goalHandle = activeGoals_.back();
            }
        }
        if (!goalHandle) {
            return false;
        }

        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        goalHandle->succeed(result);
        return true;
    }

private:
    std::atomic<bool> acceptGoals_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::size_t acceptedGoalCount_{0};
    std::size_t cancelRequestCount_{0};
    std::vector<FollowJointTrajectory::Goal> receivedGoals_;
    std::vector<ServerGoalHandlePtr> activeGoals_;
    std::shared_ptr<rclcpp_action::Server<FollowJointTrajectory>> server_;
};

class ExecutorRunner {
public:
    ExecutorRunner(const rclcpp::Node::SharedPtr& serverNode,
                   const rclcpp::Node::SharedPtr& clientNode) {
        executor_.add_node(serverNode);
        executor_.add_node(clientNode);
        spinThread_ = std::thread([this]() { executor_.spin(); });
    }

    ~ExecutorRunner() {
        executor_.cancel();
        if (spinThread_.joinable()) {
            spinThread_.join();
        }
    }

    ExecutorRunner(const ExecutorRunner&) = delete;
    ExecutorRunner& operator=(const ExecutorRunner&) = delete;

private:
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::thread spinThread_;
};

bool waitUntil(const std::function<bool()>& condition, std::chrono::milliseconds timeout = 2s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (condition()) {
            return true;
        }
        std::this_thread::sleep_for(10ms);
    }
    return condition();
}

rclcpp::Node::SharedPtr makeNode() { return rclcpp::Node::make_shared("motor_client_impl_test"); }

bool publishAndSpin(const rclcpp::Node::SharedPtr& node,
                    const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& publisher,
                    const sensor_msgs::msg::JointState& message) {
    const auto discoveryDeadline = std::chrono::steady_clock::now() + 1s;
    while (publisher->get_subscription_count() == 0 &&
           std::chrono::steady_clock::now() < discoveryDeadline) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(10ms);
    }
    if (publisher->get_subscription_count() == 0) {
        return false;
    }

    for (int attempt = 0; attempt < 10; ++attempt) {
        publisher->publish(message);
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(10ms);
    }
    rclcpp::spin_some(node);
    return true;
}

TEST(MotorClientImplTest, ConstructsThroughBaseInterface) {
    std::unique_ptr<MotorClient> client = std::make_unique<MotorClientImpl>(makeNode());
    ASSERT_NE(client, nullptr);
}

TEST(MotorClientImplTest, IsDisabledByDefault) {
    MotorClientImpl client(makeNode());
    EXPECT_FALSE(client.isEnabled());
}

TEST(MotorClientImplTest, EnableSetsEnabledState) {
    MotorClientImpl client(makeNode());
    EXPECT_TRUE(client.enable());
    EXPECT_TRUE(client.isEnabled());
    EXPECT_TRUE(client.getState().servo_enabled);
}

TEST(MotorClientImplTest, DisableFailurePreservesEnabledState) {
    MotorClientImpl client(makeNode(), "/motor_client_impl_test/disable_no_server");
    ASSERT_TRUE(client.enable());
    ASSERT_TRUE(client.isEnabled());
    ASSERT_TRUE(client.getState().servo_enabled);

    EXPECT_FALSE(client.disable());
    EXPECT_TRUE(client.isEnabled());
    EXPECT_TRUE(client.getState().servo_enabled);
}

TEST(MotorClientImplTest, GetStateReturnsDefaultState) {
    MotorClientImpl client(makeNode());
    const MotorState state = client.getState();

    EXPECT_DOUBLE_EQ(state.position_rad, 0.0);
    EXPECT_DOUBLE_EQ(state.velocity_rad_s, 0.0);
    EXPECT_FALSE(state.connected);
    EXPECT_FALSE(state.servo_enabled);
}

TEST(MotorClientImplTest, SetVelocityReturnsFalseWhileDisabled) {
    MotorClientImpl client(makeNode());
    EXPECT_FALSE(client.setVelocity(1.0));
}

TEST(MotorClientImplTest, StopReturnsFalseWithoutActionServer) {
    MotorClientImpl client(makeNode(), "/motor_client_impl_test/stop_no_server");
    EXPECT_FALSE(client.stop());
}

TEST(MotorClientImplTest, DefaultInterfaceUsesCanonicalDaisEndpoints) {
    auto serverNode = rclcpp::Node::make_shared("motor_client_default_interface_server");
    auto clientNode = rclcpp::Node::make_shared("motor_client_default_interface_client");
    FakeTrajectoryServer server(serverNode, kDefaultDaisActionName, true);
    MotorClientImpl client(clientNode);
    auto publisher =
        clientNode->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    ExecutorRunner runner(serverNode, clientNode);

    ASSERT_TRUE(waitUntil([&publisher]() { return publisher->get_subscription_count() > 0; }));
    sensor_msgs::msg::JointState message;
    message.name = {"other_joint", "joint_dais"};
    message.position = {9.0, 1.25};
    message.velocity = {8.0, -0.5};
    for (int attempt = 0; attempt < 10; ++attempt) {
        publisher->publish(message);
        std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(waitUntil([&client]() { return client.getState().connected; }));
    EXPECT_DOUBLE_EQ(client.getState().position_rad, 1.25);
    EXPECT_DOUBLE_EQ(client.getState().velocity_rad_s, -0.5);

    ASSERT_TRUE(client.enable());
    ASSERT_TRUE(client.setVelocity(0.6));
    ASSERT_TRUE(server.waitForGoal());
    ASSERT_TRUE(server.waitForAcceptedGoal());
    const auto goal = server.receivedGoal();
    ASSERT_EQ(goal.trajectory.joint_names.size(), 1U);
    EXPECT_EQ(goal.trajectory.joint_names[0], "joint_dais");
    ASSERT_TRUE(server.succeedActiveGoal());
}

TEST(MotorClientImplTest, ReadsConfiguredJointRegardlessOfArrayIndex) {
    auto node = makeNode();
    const std::string topic = "/motor_client_impl_test/joint_order";
    MotorClientImpl client(node, kUnusedActionName, topic);
    auto publisher = node->create_publisher<sensor_msgs::msg::JointState>(topic, 10);

    sensor_msgs::msg::JointState message;
    message.name = {"other_joint", "joint_dais"};
    message.position = {9.0, 1.25};
    message.velocity = {8.0, -0.5};

    ASSERT_TRUE(publishAndSpin(node, publisher, message));
    const MotorState state = client.getState();
    EXPECT_DOUBLE_EQ(state.position_rad, 1.25);
    EXPECT_DOUBLE_EQ(state.velocity_rad_s, -0.5);
    EXPECT_TRUE(state.connected);
}

TEST(MotorClientImplTest, IgnoresMessagesWithoutConfiguredJoint) {
    auto node = makeNode();
    const std::string topic = "/motor_client_impl_test/missing_joint";
    MotorClientImpl client(node, kUnusedActionName, topic);
    auto publisher = node->create_publisher<sensor_msgs::msg::JointState>(topic, 10);

    sensor_msgs::msg::JointState message;
    message.name = {"other_joint"};
    message.position = {1.0};
    message.velocity = {2.0};

    ASSERT_TRUE(publishAndSpin(node, publisher, message));
    const MotorState state = client.getState();
    EXPECT_DOUBLE_EQ(state.position_rad, 0.0);
    EXPECT_DOUBLE_EQ(state.velocity_rad_s, 0.0);
    EXPECT_FALSE(state.connected);
}

TEST(MotorClientImplTest, HandlesMissingPosition) {
    auto node = makeNode();
    const std::string topic = "/motor_client_impl_test/missing_position";
    MotorClientImpl client(node, kUnusedActionName, topic);
    auto publisher = node->create_publisher<sensor_msgs::msg::JointState>(topic, 10);

    sensor_msgs::msg::JointState message;
    message.name = {"joint_dais"};
    message.velocity = {2.5};

    ASSERT_TRUE(publishAndSpin(node, publisher, message));
    const MotorState state = client.getState();
    EXPECT_DOUBLE_EQ(state.position_rad, 0.0);
    EXPECT_DOUBLE_EQ(state.velocity_rad_s, 2.5);
    EXPECT_TRUE(state.connected);
}

TEST(MotorClientImplTest, HandlesMissingVelocity) {
    auto node = makeNode();
    const std::string topic = "/motor_client_impl_test/missing_velocity";
    MotorClientImpl client(node, kUnusedActionName, topic);
    auto publisher = node->create_publisher<sensor_msgs::msg::JointState>(topic, 10);

    sensor_msgs::msg::JointState message;
    message.name = {"joint_dais"};
    message.position = {-1.75};

    ASSERT_TRUE(publishAndSpin(node, publisher, message));
    const MotorState state = client.getState();
    EXPECT_DOUBLE_EQ(state.position_rad, -1.75);
    EXPECT_DOUBLE_EQ(state.velocity_rad_s, 0.0);
    EXPECT_TRUE(state.connected);
}

TEST(MotorClientImplTest, PreservesJointStateUnits) {
    auto node = makeNode();
    const std::string topic = "/motor_client_impl_test/units";
    MotorClientImpl client(node, kUnusedActionName, topic);
    auto publisher = node->create_publisher<sensor_msgs::msg::JointState>(topic, 10);

    sensor_msgs::msg::JointState message;
    message.name = {"joint_dais"};
    message.position = {3.141592653589793};
    message.velocity = {-2.0};

    ASSERT_TRUE(publishAndSpin(node, publisher, message));
    const MotorState state = client.getState();
    EXPECT_DOUBLE_EQ(state.position_rad, 3.141592653589793);
    EXPECT_DOUBLE_EQ(state.velocity_rad_s, -2.0);
}

TEST(MotorClientImplTest, SetVelocityReturnsFalseWithoutActionServer) {
    auto node = makeNode();
    MotorClientImpl client(node, "/motor_client_impl_test/no_server");
    ASSERT_TRUE(client.enable());

    bool result = true;
    EXPECT_NO_THROW(result = client.setVelocity(1.0));
    EXPECT_FALSE(result);
}

TEST(MotorClientImplTest, SetVelocityRejectsNonFiniteValues) {
    auto node = makeNode();
    MotorClientImpl client(node, "/motor_client_impl_test/invalid_velocity");
    ASSERT_TRUE(client.enable());

    EXPECT_FALSE(client.setVelocity(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(client.setVelocity(std::numeric_limits<double>::infinity()));
    EXPECT_FALSE(client.setVelocity(-std::numeric_limits<double>::infinity()));
}

TEST(MotorClientImplTest, AcceptedGoalContainsVelocityAndTracksLifecycle) {
    auto serverNode = rclcpp::Node::make_shared("motor_client_fake_accept_server");
    auto clientNode = rclcpp::Node::make_shared("motor_client_accept_client");
    const std::string actionName = "/motor_client_impl_test/accepted_goal";
    FakeTrajectoryServer server(serverNode, actionName, true);
    MotorClientImpl client(clientNode, actionName);
    ExecutorRunner runner(serverNode, clientNode);

    ASSERT_TRUE(client.enable());
    ASSERT_TRUE(client.setVelocity(1.5));
    ASSERT_TRUE(server.waitForGoal());
    ASSERT_TRUE(server.waitForAcceptedGoal());
    ASSERT_TRUE(
        waitUntil([&client]() { return MotorClientImplTestAccess::hasActiveGoal(client); }));

    const auto goal = server.receivedGoal();
    ASSERT_EQ(goal.trajectory.joint_names.size(), 1U);
    EXPECT_EQ(goal.trajectory.joint_names[0], "joint_dais");
    ASSERT_EQ(goal.trajectory.points.size(), 1U);
    const auto& point = goal.trajectory.points[0];
    EXPECT_TRUE(point.positions.empty());
    ASSERT_EQ(point.velocities.size(), 1U);
    EXPECT_DOUBLE_EQ(point.velocities[0], 1.5);
    EXPECT_TRUE(point.time_from_start.sec > 0 || point.time_from_start.nanosec > 0U);

    ASSERT_TRUE(server.succeedActiveGoal());
    EXPECT_TRUE(
        waitUntil([&client]() { return !MotorClientImplTestAccess::hasActiveGoal(client); }));
}

TEST(MotorClientImplTest, SendsNegativeVelocityWithoutConversion) {
    auto serverNode = rclcpp::Node::make_shared("motor_client_fake_negative_server");
    auto clientNode = rclcpp::Node::make_shared("motor_client_negative_client");
    const std::string actionName = "/motor_client_impl_test/negative_goal";
    FakeTrajectoryServer server(serverNode, actionName, true);
    MotorClientImpl client(clientNode, actionName);
    ExecutorRunner runner(serverNode, clientNode);

    ASSERT_TRUE(client.enable());
    ASSERT_TRUE(client.setVelocity(-0.7));
    ASSERT_TRUE(server.waitForGoal());

    const auto goal = server.receivedGoal();
    ASSERT_EQ(goal.trajectory.points.size(), 1U);
    ASSERT_EQ(goal.trajectory.points[0].velocities.size(), 1U);
    EXPECT_DOUBLE_EQ(goal.trajectory.points[0].velocities[0], -0.7);

    ASSERT_TRUE(server.waitForAcceptedGoal());
    ASSERT_TRUE(server.succeedActiveGoal());
}

TEST(MotorClientImplTest, RejectedGoalDoesNotBecomeActive) {
    auto serverNode = rclcpp::Node::make_shared("motor_client_fake_reject_server");
    auto clientNode = rclcpp::Node::make_shared("motor_client_reject_client");
    const std::string actionName = "/motor_client_impl_test/rejected_goal";
    FakeTrajectoryServer server(serverNode, actionName, false);
    MotorClientImpl client(clientNode, actionName);
    ExecutorRunner runner(serverNode, clientNode);

    ASSERT_TRUE(client.enable());
    ASSERT_TRUE(client.setVelocity(0.4));
    ASSERT_TRUE(server.waitForGoal());
    std::this_thread::sleep_for(100ms);
    EXPECT_FALSE(MotorClientImplTestAccess::hasActiveGoal(client));
}

TEST(MotorClientImplTest, RejectedReplacementGoalPreservesActiveGoalForStop) {
    auto serverNode = rclcpp::Node::make_shared("motor_client_rejected_replacement_server");
    auto clientNode = rclcpp::Node::make_shared("motor_client_rejected_replacement_client");
    const std::string actionName = "/motor_client_impl_test/rejected_replacement";
    FakeTrajectoryServer server(serverNode, actionName, true);
    MotorClientImpl client(clientNode, actionName);
    ExecutorRunner runner(serverNode, clientNode);

    ASSERT_TRUE(client.enable());
    ASSERT_TRUE(client.setVelocity(0.8));
    ASSERT_TRUE(server.waitForGoalCount(1));
    ASSERT_TRUE(server.waitForAcceptedGoalCount(1));
    ASSERT_TRUE(
        waitUntil([&client]() { return MotorClientImplTestAccess::hasActiveGoal(client); }));

    server.setAcceptGoals(false);
    ASSERT_TRUE(client.setVelocity(0.4));
    ASSERT_TRUE(server.waitForGoalCount(2));
    std::this_thread::sleep_for(100ms);
    ASSERT_TRUE(MotorClientImplTestAccess::hasActiveGoal(client));

    server.setAcceptGoals(true);
    ASSERT_TRUE(client.stop());
    ASSERT_TRUE(server.waitForCancelCount(1));
    ASSERT_TRUE(server.waitForGoalCount(3));

    const auto stopGoal = server.receivedGoal(2);
    ASSERT_EQ(stopGoal.trajectory.points.size(), 1U);
    ASSERT_EQ(stopGoal.trajectory.points[0].velocities.size(), 1U);
    EXPECT_DOUBLE_EQ(stopGoal.trajectory.points[0].velocities[0], 0.0);
}

TEST(MotorClientImplTest, StopCancelsActiveGoalAndSendsZeroVelocity) {
    auto serverNode = rclcpp::Node::make_shared("motor_client_stop_active_server");
    auto clientNode = rclcpp::Node::make_shared("motor_client_stop_active_client");
    const std::string actionName = "/motor_client_impl_test/stop_active";
    FakeTrajectoryServer server(serverNode, actionName, true);
    MotorClientImpl client(clientNode, actionName);
    ExecutorRunner runner(serverNode, clientNode);

    ASSERT_TRUE(client.enable());
    ASSERT_TRUE(client.setVelocity(1.5));
    ASSERT_TRUE(server.waitForGoalCount(1));
    ASSERT_TRUE(server.waitForAcceptedGoalCount(1));
    ASSERT_TRUE(
        waitUntil([&client]() { return MotorClientImplTestAccess::hasActiveGoal(client); }));

    ASSERT_TRUE(client.stop());
    ASSERT_TRUE(server.waitForCancelCount(1));
    ASSERT_TRUE(server.waitForGoalCount(2));

    const auto movingGoal = server.receivedGoal(0);
    const auto stopGoal = server.receivedGoal(1);
    ASSERT_EQ(movingGoal.trajectory.points.size(), 1U);
    ASSERT_EQ(movingGoal.trajectory.points[0].velocities.size(), 1U);
    EXPECT_DOUBLE_EQ(movingGoal.trajectory.points[0].velocities[0], 1.5);
    ASSERT_EQ(stopGoal.trajectory.points.size(), 1U);
    EXPECT_TRUE(stopGoal.trajectory.points[0].positions.empty());
    ASSERT_EQ(stopGoal.trajectory.points[0].velocities.size(), 1U);
    EXPECT_DOUBLE_EQ(stopGoal.trajectory.points[0].velocities[0], 0.0);
    EXPECT_EQ(stopGoal.trajectory.points[0].time_from_start.sec, 0);
    EXPECT_EQ(stopGoal.trajectory.points[0].time_from_start.nanosec, 500000000U);
}

TEST(MotorClientImplTest, StopWithoutActiveGoalStillSendsZeroVelocity) {
    auto serverNode = rclcpp::Node::make_shared("motor_client_stop_idle_server");
    auto clientNode = rclcpp::Node::make_shared("motor_client_stop_idle_client");
    const std::string actionName = "/motor_client_impl_test/stop_idle";
    FakeTrajectoryServer server(serverNode, actionName, true);
    MotorClientImpl client(clientNode, actionName);
    ExecutorRunner runner(serverNode, clientNode);

    ASSERT_TRUE(client.stop());
    ASSERT_TRUE(server.waitForGoal());

    const auto stopGoal = server.receivedGoal();
    ASSERT_EQ(stopGoal.trajectory.points.size(), 1U);
    ASSERT_EQ(stopGoal.trajectory.points[0].velocities.size(), 1U);
    EXPECT_DOUBLE_EQ(stopGoal.trajectory.points[0].velocities[0], 0.0);
}

TEST(MotorClientImplTest, ConsecutiveStopsRemainSafe) {
    auto serverNode = rclcpp::Node::make_shared("motor_client_repeated_stop_server");
    auto clientNode = rclcpp::Node::make_shared("motor_client_repeated_stop_client");
    const std::string actionName = "/motor_client_impl_test/repeated_stop";
    FakeTrajectoryServer server(serverNode, actionName, true);
    MotorClientImpl client(clientNode, actionName);
    ExecutorRunner runner(serverNode, clientNode);

    ASSERT_TRUE(client.stop());
    ASSERT_TRUE(server.waitForGoalCount(1, 10s));
    ASSERT_TRUE(server.waitForAcceptedGoalCount(1, 10s));
    ASSERT_TRUE(
        waitUntil([&client]() { return MotorClientImplTestAccess::hasActiveGoal(client); }, 10s));

    EXPECT_TRUE(client.stop());
    EXPECT_TRUE(server.waitForCancelCount(1, 10s));
    ASSERT_TRUE(server.waitForGoalCount(2, 10s));
    for (std::size_t index = 0; index < 2; ++index) {
        const auto goal = server.receivedGoal(index);
        ASSERT_EQ(goal.trajectory.points.size(), 1U);
        ASSERT_EQ(goal.trajectory.points[0].velocities.size(), 1U);
        EXPECT_DOUBLE_EQ(goal.trajectory.points[0].velocities[0], 0.0);
    }
}

TEST(MotorClientImplTest, StopDoesNotOverwriteFeedbackVelocity) {
    auto serverNode = rclcpp::Node::make_shared("motor_client_stop_state_server");
    auto clientNode = rclcpp::Node::make_shared("motor_client_stop_state_client");
    const std::string actionName = "/motor_client_impl_test/stop_state";
    const std::string jointStateTopic = "/motor_client_impl_test/stop_state/joint_states";
    FakeTrajectoryServer server(serverNode, actionName, true);
    MotorClientImpl client(clientNode, actionName, jointStateTopic);
    auto publisher =
        clientNode->create_publisher<sensor_msgs::msg::JointState>(jointStateTopic, 10);
    ExecutorRunner runner(serverNode, clientNode);

    ASSERT_TRUE(waitUntil([&publisher]() { return publisher->get_subscription_count() > 0; }));
    sensor_msgs::msg::JointState message;
    message.name = {"joint_dais"};
    message.position = {0.25};
    message.velocity = {0.8};
    for (int attempt = 0; attempt < 10; ++attempt) {
        publisher->publish(message);
        std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(waitUntil([&client]() { return client.getState().connected; }));
    ASSERT_DOUBLE_EQ(client.getState().velocity_rad_s, 0.8);

    ASSERT_TRUE(client.stop());
    ASSERT_TRUE(server.waitForGoal());
    EXPECT_DOUBLE_EQ(client.getState().velocity_rad_s, 0.8);
}

TEST(MotorClientImplTest, DisabledClientDoesNotSendVelocityGoal) {
    auto serverNode = rclcpp::Node::make_shared("motor_client_disabled_gate_server");
    auto clientNode = rclcpp::Node::make_shared("motor_client_disabled_gate_client");
    const std::string actionName = "/motor_client_impl_test/disabled_gate";
    FakeTrajectoryServer server(serverNode, actionName, true);
    MotorClientImpl client(clientNode, actionName);
    ExecutorRunner runner(serverNode, clientNode);

    EXPECT_FALSE(client.setVelocity(1.0));
    EXPECT_FALSE(server.waitForGoal(200ms));
}

TEST(MotorClientImplTest, DisableStopsMotorThenClearsEnabledState) {
    auto serverNode = rclcpp::Node::make_shared("motor_client_disable_server");
    auto clientNode = rclcpp::Node::make_shared("motor_client_disable_client");
    const std::string actionName = "/motor_client_impl_test/disable";
    FakeTrajectoryServer server(serverNode, actionName, true);
    MotorClientImpl client(clientNode, actionName);
    ExecutorRunner runner(serverNode, clientNode);

    ASSERT_TRUE(client.enable());
    ASSERT_TRUE(client.getState().servo_enabled);
    ASSERT_TRUE(client.setVelocity(0.9));
    ASSERT_TRUE(server.waitForGoalCount(1, 10s));
    ASSERT_TRUE(server.waitForAcceptedGoalCount(1, 10s));
    ASSERT_TRUE(
        waitUntil([&client]() { return MotorClientImplTestAccess::hasActiveGoal(client); }, 10s));

    ASSERT_TRUE(client.disable());
    ASSERT_TRUE(server.waitForCancelCount(1, 10s));
    ASSERT_TRUE(server.waitForGoalCount(2, 10s));
    const auto stopGoal = server.receivedGoal(1);
    ASSERT_EQ(stopGoal.trajectory.points.size(), 1U);
    ASSERT_EQ(stopGoal.trajectory.points[0].velocities.size(), 1U);
    EXPECT_DOUBLE_EQ(stopGoal.trajectory.points[0].velocities[0], 0.0);
    EXPECT_FALSE(client.isEnabled());
    EXPECT_FALSE(client.getState().servo_enabled);
    EXPECT_FALSE(client.setVelocity(0.5));
    EXPECT_FALSE(server.waitForGoalCount(3, 200ms));
}

TEST(MotorClientImplTest, StopCancelsInFlightGoalBeforeAcceptance) {
    auto serverNode = rclcpp::Node::make_shared("motor_client_race_fix_server");
    auto clientNode = rclcpp::Node::make_shared("motor_client_race_fix_client");
    const std::string actionName = "/motor_client_impl_test/race_fix";
    FakeTrajectoryServer server(serverNode, actionName, true);
    MotorClientImpl client(clientNode, actionName);
    ExecutorRunner runner(serverNode, clientNode);

    ASSERT_TRUE(client.enable());
    ASSERT_TRUE(client.setVelocity(1.5));
    ASSERT_TRUE(server.waitForGoalCount(1, 10s));
    ASSERT_TRUE(client.stop());
    ASSERT_TRUE(server.waitForGoalCount(2, 10s));

    const auto stopGoal = server.receivedGoal(1);
    ASSERT_EQ(stopGoal.trajectory.points.size(), 1U);
    ASSERT_EQ(stopGoal.trajectory.points[0].velocities.size(), 1U);
    EXPECT_DOUBLE_EQ(stopGoal.trajectory.points[0].velocities[0], 0.0);
}

}  // namespace
}  // namespace omr_controller
