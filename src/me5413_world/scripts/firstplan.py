#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import tf


def create_goal(x, y, yaw=0.0):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # 将偏航角转换为四元数
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    return goal


def sequential_goals():
    rospy.init_node('sequential_goal_publisher')

    # 创建一个 Action 客户端
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()

    # 定义一系列目标点
    waypoints = [
#        (21.2, 0, 0.0),
#        (20.6, -7.13, 0.0),
#        (20.6, -10.72, 0.0),
        (19.6, -21.8, 0.0)
    ]
    print(waypoints)

    for i, (x, y, yaw) in enumerate(waypoints):
        goal = create_goal(x, y, yaw)
        rospy.loginfo(f"Sending goal {i + 1}/{len(waypoints)}: x={x}, y={y}, yaw={yaw}")

        # 发送目标并等待结果
        client.send_goal(goal)

        # 等待机器人到达目标点 (timeout 设为 0 表示无限等待)
        client.wait_for_result()

        result = client.get_state()
        if result == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Goal {i + 1} reached successfully!")
        else:
            rospy.logwarn(f"Failed to reach goal {i + 1}!")

        # 在两个目标点之间稍作停留
        rospy.sleep(1.0)

        print(waypoints)

if __name__ == '__main__':
    try:
        sequential_goals()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")