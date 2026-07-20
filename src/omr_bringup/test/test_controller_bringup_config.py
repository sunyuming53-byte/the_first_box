#!/usr/bin/env python3

"""Static regression tests for ros2_control manager/controller mappings."""

import ast
import re
import unittest
from dataclasses import dataclass
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"
LAUNCH_DIR = PACKAGE_ROOT / "launch"


@dataclass(frozen=True)
class Subsystem:
    manager: str
    config: str
    broadcaster: str
    command_controller: str

    @property
    def manager_path(self):
        return f"/{self.manager}"


SUBSYSTEMS = {
    "arm": Subsystem(
        manager="controller_manager",
        config="realman_controllers.yaml",
        broadcaster="joint_state_broadcaster",
        command_controller="joint_trajectory_controller",
    ),
    "dais": Subsystem(
        manager="dais_controller_manager",
        config="dais_controllers.yaml",
        broadcaster="dais_joint_state_broadcaster",
        command_controller="dais_joint_trajectory_controller",
    ),
    "m65": Subsystem(
        manager="m65_controller_manager",
        config="m65_controllers.yaml",
        broadcaster="m65_joint_state_broadcaster",
        command_controller="diff_drive_controller",
    ),
}

EXPECTED_CONTROLLER_TYPES = {
    "joint_state_broadcaster": "joint_state_broadcaster/JointStateBroadcaster",
    "joint_trajectory_controller": "joint_trajectory_controller/JointTrajectoryController",
    "dais_joint_state_broadcaster": "joint_state_broadcaster/JointStateBroadcaster",
    "dais_joint_trajectory_controller": (
        "joint_trajectory_controller/JointTrajectoryController"
    ),
    "m65_joint_state_broadcaster": "joint_state_broadcaster/JointStateBroadcaster",
    "diff_drive_controller": "diff_drive_controller/DiffDriveController",
}

KEY_PATTERN = re.compile(r"^( *)([A-Za-z0-9_]+):(?:[ \t]*(.*))?$")
FORBIDDEN_JOINT_STATE_TOPICS = ("/dais/joint_states", "/m65/joint_states")


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


def parse_controller_nodes(path):
    """Return manager-to-config mappings and (controller, manager) spawners."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    managers = {}
    spawners = []

    for call in (node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        if not isinstance(call.func, ast.Name):
            continue

        if call.func.id == "ExecuteProcess":
            command = _literal_strings(_keyword(call, "cmd"))
            if command[:4] != ["ros2", "run", "controller_manager", "spawner"]:
                continue
            for manager_option in ("--controller-manager", "-c"):
                if manager_option in command:
                    manager_index = command.index(manager_option) + 1
                    if len(command) > 4 and manager_index < len(command):
                        spawners.append((command[4], command[manager_index]))
                    break
            continue
        if call.func.id != "Node":
            continue
        if _literal_string(_keyword(call, "package")) != "controller_manager":
            continue

        executable = _literal_string(_keyword(call, "executable"))
        if executable == "ros2_control_node":
            manager = _literal_string(_keyword(call, "name"))
            config_files = [
                value
                for value in _literal_strings(_keyword(call, "parameters"))
                if value.endswith("_controllers.yaml")
            ]
            if manager and len(config_files) == 1:
                managers[manager] = config_files[0]
        elif executable == "spawner":
            arguments = _literal_strings(_keyword(call, "arguments"))
            if "--controller-manager" not in arguments:
                continue
            manager_index = arguments.index("--controller-manager") + 1
            if arguments and manager_index < len(arguments):
                spawners.append((arguments[0], arguments[manager_index]))

    return managers, spawners


def yaml_top_level_keys(path):
    keys = []
    for line in path.read_text(encoding="utf-8").splitlines():
        match = KEY_PATTERN.match(line)
        if match and len(match.group(1)) == 0:
            keys.append(match.group(2))
    return keys


def manager_controller_types(path, manager_root):
    """Parse controller type declarations below a manager's ros__parameters."""
    current_top_level = None
    current_controller = None
    in_manager_parameters = False
    controller_types = {}

    for line in path.read_text(encoding="utf-8").splitlines():
        match = KEY_PATTERN.match(line)
        if not match:
            continue
        indent = len(match.group(1))
        key = match.group(2)
        value = (match.group(3) or "").split("#", maxsplit=1)[0].strip()

        if indent == 0:
            current_top_level = key
            current_controller = None
            in_manager_parameters = False
        elif current_top_level == manager_root and indent == 2 and key == "ros__parameters":
            in_manager_parameters = True
        elif current_top_level == manager_root and in_manager_parameters and indent == 4:
            current_controller = key if not value else None
        elif (
            current_top_level == manager_root
            and in_manager_parameters
            and current_controller
            and indent == 6
            and key == "type"
        ):
            controller_types[current_controller] = value

    return controller_types


def controller_parameters(path, controller_name):
    """Parse scalar and list parameters from one top-level controller block."""
    current_top_level = None
    current_parameter = None
    in_parameters = False
    parameters = {}

    for line in path.read_text(encoding="utf-8").splitlines():
        key_match = KEY_PATTERN.match(line)
        if key_match:
            indent = len(key_match.group(1))
            key = key_match.group(2)
            value = (key_match.group(3) or "").split("#", maxsplit=1)[0].strip()

            if indent == 0:
                current_top_level = key
                current_parameter = None
                in_parameters = False
            elif (
                current_top_level == controller_name
                and indent == 2
                and key == "ros__parameters"
            ):
                in_parameters = True
            elif current_top_level == controller_name and in_parameters and indent == 4:
                current_parameter = key
                parameters[key] = value if value else []
            continue

        list_match = re.match(r"^ {6}-[ \t]+(.+?)[ \t]*(?:#.*)?$", line)
        if (
            list_match
            and current_top_level == controller_name
            and in_parameters
            and current_parameter
            and isinstance(parameters[current_parameter], list)
        ):
            parameters[current_parameter].append(list_match.group(1))

    return parameters


class ControllerBringupConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hardware_managers, cls.hardware_spawners = parse_controller_nodes(
            LAUNCH_DIR / "bringup.launch.py"
        )
        _, cls.simulation_spawners = parse_controller_nodes(
            LAUNCH_DIR / "simulation.launch.py"
        )

    def test_hardware_manager_names_match_yaml_roots(self):
        expected_managers = {
            subsystem.manager: subsystem.config for subsystem in SUBSYSTEMS.values()
        }
        self.assertEqual(expected_managers, self.hardware_managers)

        for name, subsystem in SUBSYSTEMS.items():
            with self.subTest(subsystem=name):
                top_level_keys = yaml_top_level_keys(CONFIG_DIR / subsystem.config)
                self.assertIn(subsystem.manager, top_level_keys)

    def test_hardware_spawners_match_declared_controllers(self):
        for name, subsystem in SUBSYSTEMS.items():
            with self.subTest(subsystem=name):
                config_path = CONFIG_DIR / subsystem.config
                controller_types = manager_controller_types(config_path, subsystem.manager)
                expected_controllers = (subsystem.broadcaster, subsystem.command_controller)

                for controller in expected_controllers:
                    self.assertEqual(
                        EXPECTED_CONTROLLER_TYPES[controller], controller_types.get(controller)
                    )
                    self.assertIn(
                        (controller, subsystem.manager_path), self.hardware_spawners
                    )

    def test_hardware_broadcaster_names_are_unique(self):
        configured_broadcasters = []
        for subsystem in SUBSYSTEMS.values():
            config_path = CONFIG_DIR / subsystem.config
            controller_types = manager_controller_types(config_path, subsystem.manager)
            configured_broadcasters.extend(
                controller
                for controller, controller_type in controller_types.items()
                if controller_type == "joint_state_broadcaster/JointStateBroadcaster"
            )

        self.assertEqual(
            len(configured_broadcasters),
            len(set(configured_broadcasters)),
            f"duplicate state broadcaster names: {configured_broadcasters}",
        )

    def test_arm_and_dais_action_names_follow_contract(self):
        action_controllers = {}
        for subsystem_name in ("arm", "dais"):
            subsystem = SUBSYSTEMS[subsystem_name]
            controller_types = manager_controller_types(
                CONFIG_DIR / subsystem.config, subsystem.manager
            )
            trajectory_controllers = [
                controller
                for controller, controller_type in controller_types.items()
                if controller_type == "joint_trajectory_controller/JointTrajectoryController"
            ]
            self.assertEqual([subsystem.command_controller], trajectory_controllers)
            action_controllers[subsystem_name] = trajectory_controllers[0]

        self.assertNotEqual(action_controllers["arm"], action_controllers["dais"])
        self.assertEqual(
            "/joint_trajectory_controller/follow_joint_trajectory",
            f"/{action_controllers['arm']}/follow_joint_trajectory",
        )
        self.assertEqual(
            "/dais_joint_trajectory_controller/follow_joint_trajectory",
            f"/{action_controllers['dais']}/follow_joint_trajectory",
        )

    def test_combined_simulation_matches_hardware_controller_names(self):
        expected_spawners = {
            (SUBSYSTEMS["arm"].broadcaster, "/controller_manager"),
            (SUBSYSTEMS["arm"].command_controller, "/controller_manager"),
            (SUBSYSTEMS["dais"].broadcaster, "/controller_manager"),
            (SUBSYSTEMS["dais"].command_controller, "/controller_manager"),
        }
        self.assertEqual(expected_spawners, set(self.simulation_spawners))

        config_path = CONFIG_DIR / "sim_controllers_combined.yaml"
        self.assertIn("controller_manager", yaml_top_level_keys(config_path))
        controller_types = manager_controller_types(config_path, "controller_manager")
        for subsystem_name in ("arm", "dais"):
            subsystem = SUBSYSTEMS[subsystem_name]
            for controller in (subsystem.broadcaster, subsystem.command_controller):
                with self.subTest(subsystem=subsystem_name, controller=controller):
                    self.assertEqual(
                        EXPECTED_CONTROLLER_TYPES[controller], controller_types.get(controller)
                    )

    def test_standalone_dais_simulation_uses_hardware_controller_names(self):
        config_path = CONFIG_DIR / "sim_controllers_dais.yaml"
        self.assertIn("controller_manager", yaml_top_level_keys(config_path))
        controller_types = manager_controller_types(config_path, "controller_manager")

        for controller in (
            SUBSYSTEMS["dais"].broadcaster,
            SUBSYSTEMS["dais"].command_controller,
        ):
            with self.subTest(controller=controller):
                self.assertEqual(
                    EXPECTED_CONTROLLER_TYPES[controller], controller_types.get(controller)
                )

    def test_joint_states_remain_aggregated(self):
        paths = list(CONFIG_DIR.glob("*controllers*.yaml")) + list(LAUNCH_DIR.glob("*.py"))
        combined_text = "\n".join(path.read_text(encoding="utf-8") for path in paths)

        for forbidden_topic in FORBIDDEN_JOINT_STATE_TOPICS:
            with self.subTest(topic=forbidden_topic):
                self.assertNotIn(forbidden_topic, combined_text)
        self.assertNotRegex(combined_text, r"use_local_topics\s*:\s*true")

    def test_dais_joint_interfaces_preserve_linear_unit_contract(self):
        parameters = controller_parameters(
            CONFIG_DIR / SUBSYSTEMS["dais"].config,
            SUBSYSTEMS["dais"].command_controller,
        )

        self.assertEqual(["joint_dais"], parameters.get("joints"))
        self.assertEqual(["joint_dais"], parameters.get("command_joints"))
        self.assertEqual(["velocity"], parameters.get("command_interfaces"))
        self.assertEqual(["position", "velocity"], parameters.get("state_interfaces"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
