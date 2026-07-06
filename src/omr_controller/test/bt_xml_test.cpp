#include <omr_controller/state_machine/bt_factory.hpp>

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>

#include <fstream>
#include <sstream>
#include <string>

// ============================================================================
// Helpers
// ============================================================================

/// Derive the XML file path from the test source location.
static std::string xml_file_path() {
    std::string path(__FILE__);
    auto pos = path.rfind('/');
    if (pos != std::string::npos) {
        path = path.substr(0, pos);
    }
    return path + "/../bt_xml/pick_and_place.xml";
}

/// Read the entire contents of the XML file into a string.
static std::string read_xml() {
    auto p = xml_file_path();
    std::ifstream f(p);
    if (!f.is_open()) {
        return {};
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// ============================================================================
// Tests
// ============================================================================

TEST(PickAndPlaceXmlTest, FileExistsAndReadable) {
    auto path = xml_file_path();
    std::ifstream f(path);
    ASSERT_TRUE(f.good()) << "Cannot open XML file: " << path;

    std::string content = read_xml();
    ASSERT_FALSE(content.empty());

    // Sanity-check expected element names are present.
    EXPECT_NE(content.find("PickAndPlace"), std::string::npos);
    EXPECT_NE(content.find("MoveArmAction"), std::string::npos);
    EXPECT_NE(content.find("GripperAction"), std::string::npos);
    EXPECT_NE(content.find("DetectObjectAction"), std::string::npos);
    EXPECT_NE(content.find("Fallback"), std::string::npos);
}

TEST(PickAndPlaceXmlTest, ParsesWithAllNodesRegistered) {
    std::string xml_text = read_xml();
    ASSERT_FALSE(xml_text.empty());

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<omr_controller::MoveArmAction>("MoveArmAction");
    factory.registerNodeType<omr_controller::GripperAction>("GripperAction");
    factory.registerNodeType<omr_controller::DetectObjectAction>("DetectObjectAction");
    factory.registerNodeType<omr_controller::WaitAction>("WaitAction");

    EXPECT_NO_THROW({ factory.createTreeFromText(xml_text); });
}

TEST(PickAndPlaceXmlTest, HasCorrectStructure) {
    std::string xml_text = read_xml();
    ASSERT_FALSE(xml_text.empty());

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<omr_controller::MoveArmAction>("MoveArmAction");
    factory.registerNodeType<omr_controller::GripperAction>("GripperAction");
    factory.registerNodeType<omr_controller::DetectObjectAction>("DetectObjectAction");
    factory.registerNodeType<omr_controller::WaitAction>("WaitAction");

    BT::Tree tree = factory.createTreeFromText(xml_text);

    // Single main behavior tree.
    ASSERT_EQ(tree.subtrees.size(), 1u);

    // Expected flattened node count:
    //   1 Sequence (root)
    //   1 MoveArmAction
    //   1 Fallback
    //   1 Sequence (branch 1)
    //   1 DetectObjectAction + 2 MoveArmAction + 2 GripperAction
    //   1 Sequence (branch 2)
    //   1 MoveArmAction + 1 DetectObjectAction + 2 MoveArmAction + 2 GripperAction
    //   = 18 nodes
    ASSERT_EQ(tree.subtrees[0]->nodes.size(), 18u);

    // Verify the tree has a valid root node.
    ASSERT_NE(tree.rootNode(), nullptr);
}
