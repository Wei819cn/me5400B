#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped, Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_goal(x, y, yaw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("导航到目标点 (%.2f, %.2f, %.2f°) 完成", x, y, math.degrees(yaw))

def force_forward(distance, speed=0.1):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cmd = Twist()
    #cmd.linear.x = -abs(speed)
    cmd.linear.x = speed
    rate = rospy.Rate(10)
    duration = distance / speed
    t0 = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - t0 < duration and not rospy.is_shutdown():
        pub.publish(cmd)
        rate.sleep()
    cmd.linear.x = 0.0
    pub.publish(cmd)
    rospy.loginfo("已直线推进 %.2f 米", distance)

if __name__ == '__main__':
    rospy.init_node('goal_with_buffer_parking')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    rospy.loginfo("请输入车位中心点坐标 x y yaw（弧度），例如：0.0 0.9 0.0")

    while not rospy.is_shutdown():
        try:
            line = input("输入车位中心 x y yaw（exit 退出）: ")
            if line.lower() in ['exit', 'quit']:
                break
            parts = line.strip().split()
            if len(parts) != 3:
                rospy.logwarn("请输入 3 个数值，如 0.0 0.9 0.0")
                continue

            x, y, yaw = map(float, parts)
            forward_dist = 0.5

            # 前方缓冲点：从目标点往朝向方向 +0.5 米
            x_buf = x - forward_dist * math.cos(yaw)
            y_buf = y - forward_dist * math.sin(yaw)

            rospy.loginfo("阶段 1：导航至缓冲点 (%.2f, %.2f)", x_buf, y_buf)
            send_goal(x_buf, y_buf, yaw)

            rospy.sleep(1.0)
            rospy.loginfo("阶段 2：直线前进 %.2f 米进入车位", forward_dist)
            force_forward(forward_dist)

        except Exception as e:
            rospy.logwarn("输入或执行时出现异常：%s", str(e))

    rospy.loginfo("泊车程序已退出")
