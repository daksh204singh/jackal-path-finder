#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rostopic

class WaypointFollower:
    def __init__(self, namespace, waypoints, wait_time=20.0, loop=True):
        self.namespace = namespace
        self.waypoints = waypoints
        self.wait_time = wait_time
        self.loop = loop
        self.current_waypoint_index = 0
        self.goal_reached_threshold = 1  # Distance in meters to consider goal reached
        self.current_pose = None
        self.first_odom_received = False
        
        # Check for odom topic
        odom_topic = f'/{namespace}/odometry/filtered' if namespace else '/odom'
        goal_topic = f'/{namespace}/move_base_simple/goal' if namespace else '/move_base_simple/goal'
        
        # Debug available topics
        rospy.loginfo(f"[{self.namespace}] Looking for odom topic: {odom_topic}")
        all_topics = rostopic.find_by_type('nav_msgs/Odometry')
        rospy.loginfo(f"[{self.namespace}] Available odometry topics: {all_topics}")
        
        # Publishers and subscribers - handle with or without leading slash
        self.pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        rospy.loginfo(f"[{self.namespace}] Subscribed to odom topic: {odom_topic}")
        rospy.loginfo(f"[{self.namespace}] Publishing goals to: {goal_topic}")
        
        rospy.loginfo(f"[{self.namespace}] Initialized with {len(waypoints)} waypoints")
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        # Log first odometry message received
        if not self.first_odom_received:
            rospy.loginfo(f"[{self.namespace}] First odometry message received!")
            self.first_odom_received = True
    
    def distance_to_current_goal(self):
        """Calculate distance from current position to current goal"""
        if self.current_pose is None:
            return float('inf')
        
        goal_x, goal_y = self.waypoints[self.current_waypoint_index]
        dx = goal_x - self.current_pose.position.x
        dy = goal_y - self.current_pose.position.y
        return math.sqrt(dx**2 + dy**2)
    
    def publish_next_goal(self):
        """Publish the next goal in the pattern"""
        if not self.waypoints:
            rospy.logwarn(f"[{self.namespace}] No waypoints available")
            return
        
        x, y = self.waypoints[self.current_waypoint_index]
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.pub.publish(goal)
        rospy.loginfo(f"[{self.namespace}] Sent goal to x={x:.2f}, y={y:.2f} (waypoint {self.current_waypoint_index+1}/{len(self.waypoints)})")
    
    def update_waypoint_index(self):
        """Move to the next waypoint, looping back to the beginning when finished if loop=True"""
        if self.loop:
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
        else:
            if self.current_waypoint_index < len(self.waypoints) - 1:
                self.current_waypoint_index += 1
            else:
                rospy.loginfo(f"[{self.namespace}] Reached final waypoint.")
                return False
        return True
    
    def timer_callback(self, event):
        # Publish the current goal, then advance to the next
        self.publish_next_goal()
        self.update_waypoint_index()
    
    def run(self):
        # Initialize node if not already done
        if not rospy.get_node_uri():
            rospy.init_node(f'{self.namespace}_waypoint_follower')
        
        # Wait a second for ROS to settle
        rospy.sleep(1.0)

        # Fire off first goal immediately
        self.publish_next_goal()
        self.update_waypoint_index()

        # Set up a timer to publish every wait_time seconds
        timer = rospy.Timer(
            rospy.Duration(self.wait_time),
            self.timer_callback
        )

        rospy.loginfo(f"[{self.namespace}] Timer started: publishing every {self.wait_time}s")
        rospy.spin()

        # Clean up timer on shutdown
        timer.shutdown()

def generate_lawn_mower_waypoints(x_min, x_max, y_min, y_max, lane_width, padding=0.0):
    """
    Generate a lawn‐mower pattern of waypoints within the defined area,
    inset by `padding` metres from each wall.
    """
    # Shrink the rectangle by `padding` on all sides
    xm = x_min + padding
    xM = x_max - padding
    ym = y_min + padding
    yM = y_max - padding

    if xm >= xM or ym >= yM:
        rospy.logwarn(f"Padding {padding}m too large for area "
                      f"[{x_min},{x_max}]x[{y_min},{y_max}] — no waypoints!")
        return []

    waypoints = []
    y_distance = yM - ym
    num_lanes = max(2, int(y_distance / lane_width) + 1)

    for i in range(num_lanes):
        y = ym + (i * y_distance / (num_lanes - 1))
        if i % 2 == 0:
            waypoints.append((xm, y))
            waypoints.append((xM, y))
        else:
            waypoints.append((xM, y))
            waypoints.append((xm, y))

    return waypoints

if __name__ == '__main__':
    try:
        rospy.init_node('robot_waypoint_follower')
        
        robot_name = rospy.get_param("~robot_name", "robot_1")
        lane_width = rospy.get_param("~lane_width", 1.2)
        
        rospy.loginfo(f"Starting waypoint follower for {robot_name}")
        
        # List available topics to help with debugging
        all_topics = rospy.get_published_topics()
        rospy.loginfo(f"Available topics: {all_topics}")
        
        if robot_name == "robot_1":
            # Top section: x=[-9,9], y=[3,9]
            waypoints = generate_lawn_mower_waypoints(-9, 9, 3, 9, lane_width=lane_width, padding=2.0)
            r = WaypointFollower(robot_name, waypoints, loop=True)
        elif robot_name == "robot_2":
            # Bottom left section: x=[-9,0], y=[-9,-2]
            waypoints = generate_lawn_mower_waypoints(-9, 0, -9, -2, lane_width=lane_width, padding=2.0)
            r = WaypointFollower(robot_name, waypoints, loop=True)
        elif robot_name == "robot_3":
            # Bottom right section: x=[0,9], y=[-9,-2]
            waypoints = generate_lawn_mower_waypoints(0, 9, -9, -2, lane_width=lane_width, padding=2.0)
            r = WaypointFollower(robot_name, waypoints, loop=True)
        
        r.run()
    except rospy.ROSInterruptException:
        pass