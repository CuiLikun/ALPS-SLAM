"""Static checks; these do not replace roslaunch/catkin or a Gazebo run."""
import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class ConfigurationTests(unittest.TestCase):
    def test_project_xml_and_new_python_parse(self):
        for path in ROOT.rglob("*"):
            if "third_party" in path.parts or ".git" in path.parts:
                continue
            if path.name == "package.xml" or path.suffix in (".launch", ".xacro"):
                with self.subTest(path=str(path)):
                    ET.parse(path)
        for path in (ROOT / "alps_bringup/scripts").glob("*.py"):
            ast.parse(path.read_text(encoding="utf-8"))

    def test_local_planner_remaps_real_subscriptions(self):
        launch = ET.parse(ROOT / "local_planner/launch/local_planner.launch")
        for node in launch.findall("node"):
            remaps = {r.get("from"): r.get("to") for r in node.findall("remap")}
            self.assertEqual(remaps["/state_estimation"], "$(arg stateEstimationTopic)")

    def test_terrain_uses_registered_world_cloud(self):
        launch = ET.parse(ROOT / "terrain_analysis/launch/terrain_analysis.launch")
        parameters = {p.get("name"): p.get("value") for p in launch.findall("node/param")}
        self.assertEqual(parameters["registeredScanTopic"], "/registered_scan")
        self.assertEqual(parameters["odometryTopic"], "/state_estimation")

    def test_simulation_has_single_odometry_tf_owner(self):
        robot = ET.parse(ROOT / "my_simulation/urdf/my_robot.xacro")
        plugin = robot.find(".//plugin[@name='differential_drive_controller']")
        self.assertEqual(plugin.findtext("publishOdomTF"), "false")
        launch = ET.parse(ROOT / "my_simulation/launch/run_simulation.launch")
        self.assertFalse(launch.findall("node[@name='base_link_to_velodyne']"))
        self.assertFalse(launch.findall("node[@name='cmd_converter']"))

    def test_follower_is_routed_through_guard(self):
        launch = ET.parse(ROOT / "alps_bringup/launch/navigation.launch")
        argument = launch.find("include/arg[@name='cmdVelTopic']")
        self.assertEqual(argument.get("value"), "/navigation/cmd_vel")
        self.assertIsNotNone(launch.find("node[@type='navigation_guard.py']"))

    def test_tare_manual_modes_are_exclusive(self):
        launch = ET.parse(ROOT / "alps_bringup/launch/navigation.launch")
        self.assertIsNotNone(launch.find("group[@if='$(arg exploration)']/node[@pkg='tare_planner']"))
        manual = launch.find("node[@type='goal_bridge.py']")
        self.assertEqual(manual.get("unless"), "$(arg exploration)")


if __name__ == "__main__":
    unittest.main()
