#!/usr/bin/env python3
"""Adapt paired LIO-SAM scan/pose messages to CMU's map-frame contract."""
import copy

import message_filters
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf.transformations import (
    concatenate_matrices, quaternion_from_euler, quaternion_from_matrix,
    quaternion_matrix, translation_matrix, euler_from_quaternion,
)
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


def pose_matrix(position, orientation):
    return concatenate_matrices(
        translation_matrix([position.x, position.y, position.z]),
        quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w]),
    )


class LioTareBridge:
    def __init__(self):
        self.world = rospy.get_param("~world_frame", "map")
        self.sensor = rospy.get_param("~sensor_frame", "sensor")
        self.vehicle = rospy.get_param("~vehicle_frame", "vehicle")
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.state_pub = rospy.Publisher("/state_estimation", Odometry, queue_size=5)
        self.scan_state_pub = rospy.Publisher("/state_estimation_at_scan", Odometry, queue_size=5)
        self.scan_pub = rospy.Publisher("/registered_scan", PointCloud2, queue_size=2)
        self.odom_sub = message_filters.Subscriber(
            rospy.get_param("~odometry_topic", "/lio_sam/mapping/odometry"), Odometry)
        self.cloud_sub = message_filters.Subscriber(
            rospy.get_param("~cloud_topic", "/lio_sam/mapping/cloud_registered_raw"), PointCloud2)
        # The mapper publishes both outputs with the same scan stamp.
        self.sync = message_filters.TimeSynchronizer([self.odom_sub, self.cloud_sub], 30)
        self.sync.registerCallback(self.callback)

    def callback(self, odom, cloud):
        if not odom.header.frame_id or odom.header.frame_id != cloud.header.frame_id:
            rospy.logerr_throttle(5, "LIO pose and registered cloud must use the same nonempty frame")
            return
        try:
            transform = self.buffer.lookup_transform(
                self.world, cloud.header.frame_id, cloud.header.stamp, rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(5, "Waiting for scan-time world transform: %s", exc)
            return
        matrix = concatenate_matrices(
            pose_matrix(transform.transform.translation, transform.transform.rotation),
            pose_matrix(odom.pose.pose.position, odom.pose.pose.orientation))
        quat = quaternion_from_matrix(matrix)
        state = Odometry()
        state.header.stamp = odom.header.stamp
        state.header.frame_id = self.world
        # Mapping pose is the lidar pose, not base_link. Mapping covariance
        # contains internal flags, so do not forward it as a pose covariance.
        state.child_frame_id = self.sensor
        state.pose.pose.position.x, state.pose.pose.position.y, state.pose.pose.position.z = matrix[:3, 3]
        (state.pose.pose.orientation.x, state.pose.pose.orientation.y,
         state.pose.pose.orientation.z, state.pose.pose.orientation.w) = quat
        registered = do_transform_cloud(cloud, transform)
        registered.header.stamp = cloud.header.stamp
        registered.header.frame_id = self.world
        sensor_tf = TransformStamped()
        sensor_tf.header = copy.deepcopy(state.header)
        sensor_tf.child_frame_id = self.sensor
        sensor_tf.transform.translation.x = state.pose.pose.position.x
        sensor_tf.transform.translation.y = state.pose.pose.position.y
        sensor_tf.transform.translation.z = state.pose.pose.position.z
        sensor_tf.transform.rotation = copy.deepcopy(state.pose.pose.orientation)
        vehicle_tf = copy.deepcopy(sensor_tf)
        vehicle_tf.child_frame_id = self.vehicle
        # CMU local paths are yaw-aligned, with origin at the lidar (XY offset 0).
        yaw_quat = quaternion_from_euler(0, 0, euler_from_quaternion(quat)[2])
        (vehicle_tf.transform.rotation.x, vehicle_tf.transform.rotation.y,
         vehicle_tf.transform.rotation.z, vehicle_tf.transform.rotation.w) = yaw_quat
        self.broadcaster.sendTransform([sensor_tf, vehicle_tf])
        self.state_pub.publish(state)
        self.scan_state_pub.publish(state)
        self.scan_pub.publish(registered)


if __name__ == "__main__":
    rospy.init_node("lio_tare_bridge")
    bridge = LioTareBridge()
    rospy.spin()
