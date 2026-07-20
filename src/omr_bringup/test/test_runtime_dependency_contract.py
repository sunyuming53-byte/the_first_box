#!/usr/bin/env python3

"""Static regression tests for bringup runtime dependencies and launch switches."""

import ast
import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
BRINGUP_LAUNCH = PACKAGE_ROOT / "launch" / "bringup.launch.py"
PACKAGE_XML = PACKAGE_ROOT / "package.xml"
DOCKERFILE = WORKSPACE_ROOT / "Dockerfile"

EXPECTED_EXEC_DEPENDS = {
    "controller_manager",
    "diff_drive_controller",
    "foxglove_bridge",
    "joint_state_broadcaster",
    "joint_trajectory_controller",
    "omr_controller",
    "omr_lio",
    "realsense2_camera",
    "robot_state_publisher",
}

EXPECTED_DOCKER_PACKAGES = {
    "ros-humble-diff-drive-controller",
    "ros-humble-foxglove-bridge",
    "ros-humble-realsense2-camera",
    "ros-humble-ros2controlcli",
}

EXPECTED_SWITCH_DEFAULTS = {
    "launch_arm": "true",
    "launch_calib": "false",
    "launch_camera": "true",
    "launch_dais": "true",
    "launch_diagnostics": "true",
    "launch_door_trajectory": "false",
    "launch_foxglove": "true",
    "launch_m65": "true",
    "launch_m65_lio": "true",
    "launch_moveit": "false",
    "launch_remote_control": "true",
}


def _keyword(call, name):
    for keyword in call.keywords:
        if keyword.arg == name:
            return keyword.value
    return None


def _literal_string(node):
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _literal_strings(node):
    if node is None:
        return []
    return [
        child.value
        for child in ast.walk(node)
        if isinstance(child, ast.Constant) and isinstance(child.value, str)
    ]


def _call_name(call):
    if isinstance(call.func, ast.Name):
        return call.func.id
    return None


def _launch_configuration_name(node):
    if node is None:
        return None
    names = []
    for child in ast.walk(node):
        if not isinstance(child, ast.Call) or _call_name(child) != "LaunchConfiguration":
            continue
        if child.args:
            name = _literal_string(child.args[0])
            if name:
                names.append(name)
    if len(names) != 1:
        return None
    return names[0]


def _node_key(call):
    package = _literal_string(_keyword(call, "package"))
    executable = _literal_string(_keyword(call, "executable"))
    name = _literal_string(_keyword(call, "name"))
    arguments = _literal_strings(_keyword(call, "arguments"))
    if package == "controller_manager" and executable == "spawner" and arguments:
        name = arguments[0]
    return package, executable, name


def _execute_process_key(call):
    if _call_name(call) != "ExecuteProcess":
        return None

    command = _literal_strings(_keyword(call, "cmd"))
    if command[:4] != ["ros2", "run", "controller_manager", "spawner"]:
        return None
    if len(command) <= 4:
        return None
    return "controller_manager", "spawner", command[4]


def _docker_stage(text, stage_name, next_stage_name):
    start_pattern = rf"^FROM .+ AS {re.escape(stage_name)}$"
    end_pattern = rf"^FROM .+ AS {re.escape(next_stage_name)}$"
    start = re.search(start_pattern, text, flags=re.MULTILINE)
    end = re.search(end_pattern, text, flags=re.MULTILINE)
    if not start or not end or start.end() >= end.start():
        raise AssertionError(f"could not isolate Docker stage {stage_name}")
    return text[start.end() : end.start()]


class RuntimeDependencyContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch_tree = ast.parse(
            BRINGUP_LAUNCH.read_text(encoding="utf-8"),
            filename=str(BRINGUP_LAUNCH),
        )

    def test_package_manifest_declares_direct_runtime_dependencies(self):
        root = ET.parse(PACKAGE_XML).getroot()
        exec_depends = {element.text for element in root.findall("exec_depend")}
        self.assertTrue(EXPECTED_EXEC_DEPENDS.issubset(exec_depends))

    def test_optional_nodes_use_their_launch_switches(self):
        expected_conditions = {
            (
                "robot_state_publisher",
                "robot_state_publisher",
                "robot_state_publisher",
            ): "launch_arm",
            ("controller_manager", "ros2_control_node", "controller_manager"): "launch_arm",
            ("controller_manager", "spawner", "joint_state_broadcaster"): "launch_arm",
            ("controller_manager", "spawner", "joint_trajectory_controller"): "launch_arm",
            (
                "controller_manager",
                "ros2_control_node",
                "dais_controller_manager",
            ): "launch_dais",
            (
                "robot_state_publisher",
                "robot_state_publisher",
                "dais_robot_state_publisher",
            ): "launch_dais",
            (
                "controller_manager",
                "spawner",
                "dais_joint_state_broadcaster",
            ): "launch_dais",
            (
                "controller_manager",
                "spawner",
                "dais_joint_trajectory_controller",
            ): "launch_dais",
            (
                "controller_manager",
                "ros2_control_node",
                "m65_controller_manager",
            ): "launch_m65",
            (
                "robot_state_publisher",
                "robot_state_publisher",
                "m65_robot_state_publisher",
            ): "launch_m65",
            (
                "controller_manager",
                "spawner",
                "m65_joint_state_broadcaster",
            ): "launch_m65",
            ("controller_manager", "spawner", "diff_drive_controller"): "launch_m65",
            ("realsense2_camera", "realsense2_camera_node", "camera"): "launch_camera",
            ("omr_controller", "calib_node", "calib_node"): "launch_calib",
            (
                "omr_controller",
                "robot_diagnostics",
                "robot_diagnostics",
            ): "launch_diagnostics",
            (
                "omr_controller",
                "remote_control",
                "remote_control",
            ): "launch_remote_control",
            ("foxglove_bridge", "foxglove_bridge", "foxglove_bridge"): "launch_foxglove",
        }

        actual_conditions = {}
        for call in (node for node in ast.walk(self.launch_tree) if isinstance(node, ast.Call)):
            if _call_name(call) == "Node":
                key = _node_key(call)
                if key in expected_conditions:
                    actual_conditions[key] = _launch_configuration_name(
                        _keyword(call, "condition")
                    )
                continue

            if _call_name(call) != "TimerAction":
                continue

            condition_name = _launch_configuration_name(_keyword(call, "condition"))
            for child in ast.walk(call):
                if not isinstance(child, ast.Call):
                    continue
                key = _execute_process_key(child)
                if key in expected_conditions:
                    actual_conditions[key] = condition_name

        self.assertEqual(expected_conditions, actual_conditions)

    def test_optional_includes_use_their_launch_switches(self):
        expected_conditions = {
            "m65_lio_nav.launch.py": "launch_m65_lio",
            "controller.launch.py": "launch_door_trajectory",
            "move_group.launch.py": "launch_moveit",
        }
        actual_conditions = {}

        for call in (
            node
            for node in ast.walk(self.launch_tree)
            if isinstance(node, ast.Call)
            and _call_name(node) == "IncludeLaunchDescription"
        ):
            strings = _literal_strings(call)
            for launch_file in expected_conditions:
                if launch_file in strings:
                    actual_conditions[launch_file] = _launch_configuration_name(
                        _keyword(call, "condition")
                    )

        self.assertEqual(expected_conditions, actual_conditions)

    def test_all_launch_switches_are_declared(self):
        declared = set()
        referenced = set()

        for call in (node for node in ast.walk(self.launch_tree) if isinstance(node, ast.Call)):
            if _call_name(call) == "DeclareLaunchArgument" and call.args:
                name = _literal_string(call.args[0])
                if name and name.startswith("launch_"):
                    declared.add(name)
            elif _call_name(call) == "LaunchConfiguration" and call.args:
                name = _literal_string(call.args[0])
                if name and name.startswith("launch_"):
                    referenced.add(name)

        self.assertTrue(referenced)
        self.assertTrue(referenced.issubset(declared))

    def test_launch_switch_defaults_match_the_documented_workflows(self):
        defaults = {}
        for call in (
            node
            for node in ast.walk(self.launch_tree)
            if isinstance(node, ast.Call)
            and _call_name(node) == "DeclareLaunchArgument"
        ):
            if not call.args:
                continue
            name = _literal_string(call.args[0])
            if name and name.startswith("launch_"):
                defaults[name] = _literal_string(_keyword(call, "default_value"))

        self.assertEqual(EXPECTED_SWITCH_DEFAULTS, defaults)

    def test_docker_stages_install_external_runtime_packages(self):
        dockerfile = DOCKERFILE.read_text(encoding="utf-8")
        stages = {
            "omrobot-base-dev": _docker_stage(
                dockerfile, "omrobot-base-dev", "omrobot-base"
            ),
            "omrobot-base": _docker_stage(
                dockerfile, "omrobot-base", "omrobot-develop"
            ),
        }

        for stage_name, stage_text in stages.items():
            with self.subTest(stage=stage_name):
                for package in EXPECTED_DOCKER_PACKAGES:
                    self.assertIn(package, stage_text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
