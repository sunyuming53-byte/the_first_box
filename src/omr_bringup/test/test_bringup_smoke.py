#!/usr/bin/env python3

"""Hardware-free smoke test for the minimal omr_bringup launch workflow."""

import os
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path


DISABLED_SWITCHES = (
    "launch_arm:=false",
    "launch_dais:=false",
    "launch_m65:=false",
    "launch_camera:=false",
    "launch_calib:=false",
    "launch_door_trajectory:=false",
    "launch_moveit:=false",
    "launch_m65_lio:=false",
    "launch_foxglove:=false",
    "launch_diagnostics:=false",
    "launch_remote_control:=false",
)

FORBIDDEN_NODES = {
    "/calib_node",
    "/camera",
    "/controller_manager",
    "/dais_controller_manager",
    "/dais_robot_state_publisher",
    "/foxglove_bridge",
    "/m65_controller_manager",
    "/m65_robot_state_publisher",
    "/move_group",
    "/robot_diagnostics",
    "/robot_state_publisher",
    "/remote_control",
}

FORBIDDEN_LOG_MESSAGES = (
    "package not found",
    "executable not found",
    "caught exception in launch",
)


class BringupSmokeTest(unittest.TestCase):
    def test_minimal_launch_disables_optional_nodes_without_errors(self):
        environment = os.environ.copy()
        environment["ROS_DOMAIN_ID"] = "148"
        environment["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

        with tempfile.TemporaryDirectory(prefix="omr_bringup_smoke_") as temp_dir:
            log_path = Path(temp_dir) / "bringup.log"
            with log_path.open("w+", encoding="utf-8") as log_file:
                process = subprocess.Popen(
                    [
                        "ros2",
                        "launch",
                        "omr_bringup",
                        "bringup.launch.py",
                        *DISABLED_SWITCHES,
                    ],
                    env=environment,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    text=True,
                )

                try:
                    nodes = self._observe_minimal_launch(process, environment, log_file)
                    self.assertTrue(
                        FORBIDDEN_NODES.isdisjoint(nodes),
                        f"disabled nodes unexpectedly started: {sorted(FORBIDDEN_NODES & nodes)}",
                    )
                finally:
                    self._stop_process_group(process)

                log_file.flush()
                log_file.seek(0)
                log_text = log_file.read().lower()

        for message in FORBIDDEN_LOG_MESSAGES:
            with self.subTest(message=message):
                self.assertNotIn(message, log_text)

    def _observe_minimal_launch(self, process, environment, log_file):
        deadline = time.monotonic() + 3.0
        observed_nodes = set()

        while time.monotonic() < deadline:
            return_code = process.poll()
            if return_code is not None:
                if return_code == 0:
                    return observed_nodes
                log_file.flush()
                log_file.seek(0)
                self.fail(
                    f"bringup exited with code {return_code}:\n{log_file.read()}"
                )

            result = subprocess.run(
                ["ros2", "node", "list"],
                env=environment,
                check=False,
                capture_output=True,
                text=True,
                timeout=2,
            )
            if result.returncode == 0:
                observed_nodes.update(
                    line.strip() for line in result.stdout.splitlines() if line.strip()
                )
                unexpected_nodes = FORBIDDEN_NODES & observed_nodes
                if unexpected_nodes:
                    self.fail(f"disabled nodes unexpectedly started: {sorted(unexpected_nodes)}")
            time.sleep(0.25)

        return observed_nodes

    @staticmethod
    def _stop_process_group(process):
        if process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
