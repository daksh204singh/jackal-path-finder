#!/usr/bin/env python3

import rospy
import random
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class RandomGoalPublisher:
    def __init__(self, namespace, x_range, y_range, min_distance=1.0):
        self.namespace = namespace
        self.x_range = x_range
        self.y_range = y_range
        self.min_distance = min_distance

        self.pub = rospy.Publisher(f'/{namespace}/move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.Subscriber(f'/{namespace}/odom', Odometry, self.odom_callback)
        self.current_pose = None

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def generate_far_enough_goal(self):
        while True:
            x = random.uniform(self.x_range[0], self.x_range[1])
            y = random.uniform(self.y_range[0], self.y_range[1])
            if self.current_pose:
                dx = x - self.current_pose.position.x
                dy = y - self.current_pose.position.y
                distance = math.sqrt(dx**2 + dy**2)
                if distance >= self.min_distance:
                    return x, y
            else:
                return x, y  # If no odom yet, just return something

    def publish_goals(self):
        rospy.init_node(f'{self.namespace}_goal_publisher')
        rate = rospy.Rate(0.05)  # One goal every 20s
        while not rospy.is_shutdown():
            x, y = self.generate_far_enough_goal()

            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.w = 1.0

            self.pub.publish(goal)
            rospy.loginfo(f"[{self.namespace}] Sent goal to x={x:.2f}, y={y:.2f}")
            rate.sleep()


if __name__ == '__main__':
    try:
        robot_name = rospy.get_param("~robot_name", "robot_1")
        if robot_name == "robot_1":
            r = RandomGoalPublisher("robot_1", [-9, 9], [3, 9])
        elif robot_name == "robot_2":
            r = RandomGoalPublisher("robot_2", [-9, 0], [-9, -2])
        elif robot_name == "robot_3":
            r = RandomGoalPublisher("robot_3", [0, 9], [-9, -2])
        r.publish_goals()
    except rospy.ROSInterruptException:
        pass