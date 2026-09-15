#!/usr/bin/env python3
"""Transform an RViz goal into the planner's map frame (manual mode only)."""
import math
import rospy
import tf
from geometry_msgs.msg import PointStamped, PoseStamped


class GoalBridge:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.pub = rospy.Publisher("/way_point", PointStamped, queue_size=1)
        self.sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback)

    def callback(self, message):
        point = PointStamped(header=message.header, point=message.pose.position)
        try:
            point = self.listener.transformPoint("map", point)
        except tf.Exception as exc:
            rospy.logwarn("Goal rejected: %s", exc)
            return
        if not all(map(math.isfinite, (point.point.x, point.point.y, point.point.z))):
            rospy.logwarn("Goal rejected: nonfinite coordinates")
            return
        point.header.stamp = rospy.Time.now()
        self.pub.publish(point)


if __name__ == "__main__":
    rospy.init_node("goal_bridge")
    bridge = GoalBridge()
    rospy.spin()
