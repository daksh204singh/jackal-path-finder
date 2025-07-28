#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def normalize_quaternion(q):
    norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    if norm < 1e-6:
        raise ValueError("Quaternion has near-zero length!")
    q.x /= norm
    q.y /= norm
    q.z /= norm
    q.w /= norm
    return q

def main():
    rospy.init_node('goal_with_current_heading')

    pub = rospy.Publisher('/robot_1/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.sleep(1.0)

    # Wait for a valid odometry message
    while not rospy.is_shutdown():
        odom = rospy.wait_for_message('/robot_1/odometry/filtered', Odometry)
        ori = odom.pose.pose.orientation
        try:
            ori = normalize_quaternion(ori)
            break
        except ValueError:
            rospy.logwarn("Waiting for valid orientation...")

    # Use the robot's current heading in the goal
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose.position.x = -6.5
    goal.pose.position.y = 7.0
    goal.pose.position.z = 0.0
    goal.pose.orientation = ori

    pub.publish(goal)
    rospy.loginfo("Sent goal to (-6.5, 7.0) with current orientation")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass