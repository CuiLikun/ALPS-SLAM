#!/usr/bin/env python3
"""Gate follower velocity on live navigation inputs and an explicit waypoint."""
import copy
import math
import threading
import time

import rospy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2


class NavigationGuard:
    def __init__(self):
        self.timeout = float(rospy.get_param("~input_timeout", 1.0))
        self.waypoint_timeout = float(rospy.get_param("~waypoint_timeout", 5.0))
        if self.timeout <= 0 or self.waypoint_timeout < 0:
            raise ValueError("input_timeout must be positive; waypoint_timeout must be nonnegative")
        self.lock = threading.Lock()
        self.seen = {}
        self.command = Twist()
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.subs = [
            rospy.Subscriber("/navigation/cmd_vel", Twist, self.command_callback),
            rospy.Subscriber("/state_estimation", Odometry, self.mark, "odom"),
            rospy.Subscriber("/registered_scan", PointCloud2, self.cloud_callback),
            rospy.Subscriber("/path", Path, self.path_callback),
            rospy.Subscriber("/way_point", PointStamped, self.waypoint_callback),
        ]
        self.stop_event = threading.Event()
        rospy.on_shutdown(self.shutdown)
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def mark(self, message, key):
        # Reject delayed queued data, not just missing arrivals. All header
        # messages use the same ROS clock; wall time also detects paused /clock.
        age = (rospy.Time.now() - message.header.stamp).to_sec()
        with self.lock:
            if 0 <= age <= self.timeout:
                self.seen[key] = time.monotonic()
            else:
                self.seen.pop(key, None)

    def waypoint_callback(self, message):
        if message.header.frame_id == "map" and all(map(math.isfinite,
                (message.point.x, message.point.y, message.point.z))):
            self.mark(message, "waypoint")
        else:
            with self.lock:
                self.seen.pop("waypoint", None)

    def cloud_callback(self, message):
        if message.width * message.height:
            self.mark(message, "scan")
        else:
            with self.lock:
                self.seen.pop("scan", None)

    def path_callback(self, message):
        if message.poses:
            self.mark(message, "path")
        else:
            with self.lock:
                self.seen.pop("path", None)

    def command_callback(self, message):
        with self.lock:
            self.command = copy.deepcopy(message)
            self.seen["command"] = time.monotonic()

    def output(self, now):
        ready = all(key in self.seen and 0 <= now - self.seen[key] <= self.timeout
                    for key in ("odom", "scan", "path", "command"))
        # TARE's exploration_finish means coverage finished, BEFORE returning
        # home. Keep following its waypoints during the return trip.
        ready = ready and "waypoint" in self.seen
        if ready and self.waypoint_timeout:
            ready = 0 <= now - self.seen["waypoint"] <= self.waypoint_timeout
        values = (self.command.linear.x, self.command.linear.y, self.command.linear.z,
                  self.command.angular.x, self.command.angular.y, self.command.angular.z)
        return copy.deepcopy(self.command) if ready and all(map(math.isfinite, values)) else Twist()

    def run(self):
        # Wall-clock loop continues to send zero while Gazebo /clock is paused.
        while not self.stop_event.wait(0.05):
            with self.lock:
                self.pub.publish(self.output(time.monotonic()))

    def shutdown(self):
        self.stop_event.set()
        with self.lock:
            self.pub.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("navigation_guard")
    guard = NavigationGuard()
    rospy.spin()
