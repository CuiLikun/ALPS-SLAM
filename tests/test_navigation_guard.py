"""Behavioral tests of velocity gating without requiring an installed ROS."""
import importlib.util
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class Stamp:
    def __init__(self, value):
        self.value = value

    def __sub__(self, other):
        return Stamp(self.value - other.value)

    def to_sec(self):
        return self.value


def load_guard():
    modules = {"rospy": types.ModuleType("rospy")}
    modules["rospy"].Time = types.SimpleNamespace(now=lambda: Stamp(100.0))
    for name, classes in {
        "geometry_msgs": {"Twist": Twist, "PointStamped": object},
        "nav_msgs": {"Odometry": object, "Path": object},
        "sensor_msgs": {"PointCloud2": object},
    }.items():
        modules[name] = types.ModuleType(name)
        modules[name + ".msg"] = types.ModuleType(name + ".msg")
        for key, value in classes.items():
            setattr(modules[name + ".msg"], key, value)
    path = Path(__file__).resolve().parents[1] / "alps_bringup/scripts/navigation_guard.py"
    spec = importlib.util.spec_from_file_location("navigation_guard_test", path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


MODULE = load_guard()


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.guard = MODULE.NavigationGuard.__new__(MODULE.NavigationGuard)
        self.guard.timeout = 1.0
        self.guard.waypoint_timeout = 5.0
        self.guard.lock = threading.Lock()
        self.guard.seen = dict.fromkeys(("odom", "scan", "path", "command", "waypoint"), 100.0)
        self.guard.command = Twist()
        self.guard.command.linear.x = 0.3
        self.guard.command.angular.z = 0.2

    def assertStopped(self, now=100.1):
        output = self.guard.output(now)
        self.assertEqual(output.linear.x, 0.0)
        self.assertEqual(output.angular.z, 0.0)

    def test_live_inputs_pass_independent_copy(self):
        output = self.guard.output(100.1)
        self.assertEqual(output.linear.x, 0.3)
        self.assertEqual(output.angular.z, 0.2)
        output.linear.x = 5.0
        self.assertEqual(self.guard.command.linear.x, 0.3)

    def test_every_required_input_must_have_arrived(self):
        for key in tuple(self.guard.seen):
            with self.subTest(key=key):
                stamp = self.guard.seen.pop(key)
                self.assertStopped()
                self.guard.seen[key] = stamp

    def test_any_stale_sensor_or_command_stops(self):
        for key in ("odom", "scan", "path", "command"):
            with self.subTest(key=key):
                self.guard.seen[key] = 98.0
                self.assertStopped()
                self.guard.seen[key] = 100.0

    def test_exploration_waypoint_expires(self):
        self.guard.seen["waypoint"] = 94.0
        self.assertStopped()

    def test_manual_goal_persists_but_sensors_must_remain_live(self):
        self.guard.waypoint_timeout = 0.0
        self.guard.seen["waypoint"] = 1.0
        self.assertEqual(self.guard.output(100.1).linear.x, 0.3)
        self.assertStopped(102.0)

    def test_paused_clock_stops_using_wall_time(self):
        self.assertStopped(102.0)

    def test_nonfinite_command_stops(self):
        for value in (float("nan"), float("inf"), -float("inf")):
            self.guard.command.angular.z = value
            self.assertStopped()

    def test_empty_cloud_or_path_revokes_readiness(self):
        self.guard.cloud_callback(types.SimpleNamespace(width=0, height=1))
        self.assertStopped()
        self.guard.seen["scan"] = 100.0
        self.guard.path_callback(types.SimpleNamespace(poses=[]))
        self.assertStopped()

    def test_delayed_and_future_messages_are_rejected(self):
        for stamp in (90.0, 101.0):
            message = types.SimpleNamespace(header=types.SimpleNamespace(stamp=Stamp(stamp)))
            self.guard.mark(message, "odom")
            self.assertStopped()

    def test_invalid_waypoint_revokes_readiness(self):
        for frame, x in (("odom", 0.0), ("map", float("nan"))):
            self.guard.seen["waypoint"] = 100.0
            message = types.SimpleNamespace(
                header=types.SimpleNamespace(frame_id=frame, stamp=Stamp(100.0)),
                point=types.SimpleNamespace(x=x, y=0.0, z=0.0))
            self.guard.waypoint_callback(message)
            self.assertStopped()

    def test_fresh_waypoint_restores_readiness(self):
        self.guard.seen.pop("waypoint")
        message = types.SimpleNamespace(
            header=types.SimpleNamespace(frame_id="map", stamp=Stamp(100.0)),
            point=types.SimpleNamespace(x=2.0, y=1.0, z=0.0))
        with patch.object(MODULE.time, "monotonic", return_value=100.0):
            self.guard.waypoint_callback(message)
        self.assertEqual(self.guard.output(100.1).linear.x, 0.3)


if __name__ == "__main__":
    unittest.main()
