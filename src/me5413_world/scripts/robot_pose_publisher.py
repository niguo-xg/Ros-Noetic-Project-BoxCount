#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import math


class RobotPosePublisher:
    """发布机器人位姿信息的节点"""

    def __init__(self):
        rospy.init_node('robot_pose_publisher', anonymous=True)

        # 初始化tf监听器
        self.tf_listener = tf.TransformListener()

        # 订阅里程计消息
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

        # 发布机器人位置
        self.pose_pub = rospy.Publisher('/robot_pose', Point, queue_size=10)

        # 机器人位置
        self.robot_x = 0.0
        self.robot_y = 0.0

        rospy.loginfo("Robot pose publisher initialized")

    def odom_callback(self, msg):
        """处理里程计消息"""
        # 更新机器人位置
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # 直接发布位置
        self.publish_pose()

    def publish_pose(self):
        """发布机器人位置"""
        pose_msg = Point()
        pose_msg.x = self.robot_x
        pose_msg.y = self.robot_y
        pose_msg.z = 0.0

        self.pose_pub.publish(pose_msg)

    def try_get_map_pose(self):
        """尝试获取相对于地图的位置"""
        try:
            # 尝试从map到base_link的转换
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))

            # 更新位置并发布
            self.robot_x = trans[0]
            self.robot_y = trans[1]
            self.publish_pose()
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # 如果无法获取，仍然使用里程计数据
            return False

    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            # 尝试获取map坐标系中的位置
            if not self.try_get_map_pose():
                # 如果失败，使用当前里程计位置
                self.publish_pose()

            rate.sleep()


if __name__ == '__main__':
    try:
        publisher = RobotPosePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass