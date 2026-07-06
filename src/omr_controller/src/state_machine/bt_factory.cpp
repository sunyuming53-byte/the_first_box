#include "omr_controller/state_machine/bt_factory.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <chrono>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "omr_controller/state_machine/door_trajectory_action.hpp"
#include "omr_controller/types.hpp"

namespace {

std::vector<double> parseJointPositions(const std::string& input) {
    std::vector<double> result;
    std::istringstream stream(input);
    std::string token;
    while (std::getline(stream, token, ',')) {
        if (!token.empty()) {
            result.push_back(std::stod(token));
        }
    }
    return result;
}

}  // namespace

namespace omr_controller {

// ============================================================================
// MoveArmAction
// ============================================================================

MoveArmAction::MoveArmAction(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList MoveArmAction::providedPorts() {
    return {BT::InputPort<std::string>("joint_positions"), BT::InputPort<double>("speed_ratio")};
}

BT::NodeStatus MoveArmAction::tick() {
    auto* arm = config().blackboard->get<ArmClient*>("arm_client");
    if (!arm) {
        return BT::NodeStatus::FAILURE;
    }

    std::string positions_str;
    if (!getInput("joint_positions", positions_str)) {
        return BT::NodeStatus::FAILURE;
    }

    double speed_ratio = 50.0;
    getInput("speed_ratio", speed_ratio);

    JointGoal goal;
    goal.speed_ratio = speed_ratio;
    goal.positions = parseJointPositions(positions_str);

    arm->moveJoints(goal);
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// GripperAction
// ============================================================================

GripperAction::GripperAction(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList GripperAction::providedPorts() {
    return {BT::InputPort<std::string>("action"), BT::InputPort<double>("force")};
}

BT::NodeStatus GripperAction::tick() {
    auto* gripper = config().blackboard->get<GripperClient*>("gripper_client");
    if (!gripper) {
        return BT::NodeStatus::FAILURE;
    }

    std::string action;
    if (!getInput("action", action)) {
        return BT::NodeStatus::FAILURE;
    }

    double force = 50.0;
    getInput("force", force);

    if (action == "open") {
        gripper->open(force);
    } else if (action == "close") {
        gripper->close(force);
    } else {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// DetectObjectAction
// ============================================================================

DetectObjectAction::DetectObjectAction(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList DetectObjectAction::providedPorts() {
    return {BT::OutputPort<int>("detection_count")};
}

BT::NodeStatus DetectObjectAction::tick() {
    auto* vision = config().blackboard->get<VisionClient*>("vision_client");
    if (!vision) {
        return BT::NodeStatus::FAILURE;
    }

    auto results = vision->next_detection();
    if (results && !results->empty()) {
        config().blackboard->set("detection_count", static_cast<int>(results->size()));
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

// ============================================================================
// WaitAction
// ============================================================================

WaitAction::WaitAction(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList WaitAction::providedPorts() { return {BT::InputPort<int>("duration_ms")}; }

BT::NodeStatus WaitAction::tick() {
    int duration_ms = 0;
    getInput("duration_ms", duration_ms);
    if (duration_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
    }
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// build_tree
// ============================================================================

BT::Tree build_tree(const std::string& xml_text, ArmClient& arm, GripperClient& gripper,
                    VisionClient& vision,
                    rclcpp::Node::SharedPtr ros_node) {
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<MoveArmAction>("MoveArmAction");
    factory.registerNodeType<GripperAction>("GripperAction");
    factory.registerNodeType<DetectObjectAction>("DetectObjectAction");
    factory.registerNodeType<WaitAction>("WaitAction");
    factory.registerNodeType<DoorTrajectoryAction>("DoorTrajectoryAction");

    auto tree = factory.createTreeFromText(xml_text);

    tree.rootBlackboard()->set("arm_client", &arm);
    tree.rootBlackboard()->set("gripper_client", &gripper);
    tree.rootBlackboard()->set("vision_client", &vision);
    if (ros_node) {
        tree.rootBlackboard()->set("ros_node", ros_node);
    }

    return tree;
}

}  // namespace omr_controller
