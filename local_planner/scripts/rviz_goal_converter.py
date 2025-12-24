#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped

def callback(msg):
    # 将 RViz 的 PoseStamped 转换为 localPlanner 需要的 PointStamped
    point_msg = PointStamped()
    point_msg.header = msg.header
    point_msg.point = msg.pose.position
    
    # 发布到 /way_point
    pub.publish(point_msg)
    rospy.loginfo(f"已转发目标点: x={point_msg.point.x:.2f}, y={point_msg.point.y:.2f}")

if __name__ == '__main__':
    rospy.init_node('rviz_goal_converter')
    
    # 订阅 RViz 的 "2D Nav Goal" (PoseStamped)
    sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback)
    
    # 发布给 localPlanner (PointStamped)
    pub = rospy.Publisher('/way_point', PointStamped, queue_size=1)
    
    rospy.loginfo("目标点转换节点已启动！")
    rospy.loginfo("请在 RViz 中使用 '2D Nav Goal' 工具点击地图设置目标。")
    
    rospy.spin()
