#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <string>

#include "omr_controller/clients/arm_client.hpp"
#include "omr_controller/clients/gripper_client.hpp"
#include "omr_controller/clients/vision_client.hpp"

namespace omr_controller {

// ============================================================================
// Custom BehaviorTree.CPP v4 TreeNodes
// ============================================================================

/// MoveArmAction: parse a comma-separated joint_positions string and call
/// ArmClient::moveJoints().
class MoveArmAction : public BT::SyncActionNode {
public:
    MoveArmAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

/// GripperAction: call GripperClient::open() or ::close().
class GripperAction : public BT::SyncActionNode {
public:
    GripperAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

/// DetectObjectAction: call VisionClient::next_detection().
/// On success writes "detection_count" (int) to the blackboard.
class DetectObjectAction : public BT::SyncActionNode {
public:
    DetectObjectAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

/// WaitAction: sleep for duration_ms milliseconds.
class WaitAction : public BT::SyncActionNode {
public:
    WaitAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// ============================================================================
// Factory
// ============================================================================

/// Build a BehaviorTree from an XML string, registering all custom tree nodes
/// and injecting the supplied client instances into the root blackboard.
///
/// Blackboard entries set:
///   "arm_client"     -> ArmClient*
///   "gripper_client" -> GripperClient*
///   "vision_client"  -> VisionClient*
BT::Tree build_tree(const std::string& xml_text, ArmClient& arm, GripperClient& gripper,
                    VisionClient& vision);

}  // namespace omr_controller
