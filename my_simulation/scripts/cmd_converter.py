#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, TwistStamped

def callback(data):
    # 收到 TwistStamped (带时间戳)
    # 提取其中的 Twist (纯速度)
    twist_msg = data.twist
    
    # 转发给 Gazebo
    pub.publish(twist_msg)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_converter')
    
    # 1. 订阅 Local Planner 发出的带时间戳的话题
    rospy.Subscriber('/cmd_vel_stamped', TwistStamped, callback)
    
    # 2. 发布 Gazebo 能听懂的纯速度话题
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rospy.loginfo("Converter Started: TwistStamped -> Twist")
    rospy.spin()
