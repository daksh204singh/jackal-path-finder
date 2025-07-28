#!/usr/bin/env python3
import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class SmoothLineCoverageExplorer:
    def __init__(self, namespace, x_range, y_range, line_spacing=1.0):
        self.namespace = namespace
        self.x_range = x_range
        self.y_range = y_range
        self.line_spacing = line_spacing
        
        # Create action client for move_base
        self.action_client = actionlib.SimpleActionClient(f'/{namespace}/move_base', MoveBaseAction)
        rospy.loginfo(f"[{self.namespace}] Waiting for move_base action server...")
        self.action_client.wait_for_server()
        
        # Subscribe to odometry for current position
        rospy.Subscriber(f'/{namespace}/odom', Odometry, self.odom_callback)
        
        # Listen for result of move_base actions
        rospy.Subscriber(f'/{namespace}/move_base/result', MoveBaseActionResult, self.move_base_result_callback)
        
        self.current_pose = None
        self.current_goal_index = 0
        self.last_goal_status = None
        self.goals = []
        self.retries = 0
        self.max_retries = 2
        
        # Parameters for smoother movement
        self.waypoint_distance = rospy.get_param('~waypoint_distance', 0.5)  # Distance between intermediate waypoints
        
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    def move_base_result_callback(self, msg):
        self.last_goal_status = msg.status.status
        
    def generate_coverage_pattern(self):
        # Calculate the number of horizontal lines needed to cover the area
        num_lines = int((self.y_range[1] - self.y_range[0]) / self.line_spacing) + 1
        
        # Create main pattern points
        main_points = []
        for i in range(num_lines):
            y = self.y_range[0] + i * self.line_spacing
            
            # Alternate direction for each line (serpentine pattern)
            if i % 2 == 0:
                main_points.append((self.x_range[0], y))  # Start from left
                main_points.append((self.x_range[1], y))  # Go to right
            else:
                main_points.append((self.x_range[1], y))  # Start from right
                main_points.append((self.x_range[0], y))  # Go to left
        
        # Generate intermediate points for smooth movement
        pattern = []
        for i in range(len(main_points) - 1):
            x1, y1 = main_points[i]
            x2, y2 = main_points[i+1]
            
            # Add first point
            pattern.append((x1, y1))
            
            # Only add intermediate points for horizontal line segments
            # (not for vertical transitions between lines)
            if abs(y2 - y1) < 0.001:  # If on the same horizontal line
                dx = x2 - x1
                dy = y2 - y1
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Number of intermediate points
                num_points = int(distance / self.waypoint_distance)
                
                for j in range(1, num_points):
                    ratio = j / num_points
                    x = x1 + dx * ratio
                    y = y1 + dy * ratio
                    pattern.append((x, y))
        
        # Add the final point
        pattern.append(main_points[-1])
        return pattern
        
    def send_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        # Calculate orientation to face the next goal if available
        if self.current_goal_index + 1 < len(self.goals):
            next_x, next_y = self.goals[self.current_goal_index + 1]
            dx = next_x - x
            dy = next_y - y
            yaw = math.atan2(dy, dx)
        else:
            # Default orientation for last point
            yaw = 0.0
        
        # Convert to quaternion
        qz = math.sin(yaw/2.0)
        qw = math.cos(yaw/2.0)
        
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        
        # Set planner patience lower for smoother movement
        # This makes the robot more likely to try a new path if stuck
        client = rospy.Publisher(f'/{self.namespace}/move_base/DWAPlannerROS/planner_patience', rospy.Duration, queue_size=10)
        client.publish(rospy.Duration(1.0))
        
        rospy.loginfo(f"[{self.namespace}] Sending goal {self.current_goal_index+1}/{len(self.goals)}: x={x:.2f}, y={y:.2f}")
        self.action_client.send_goal(goal)
        
    def execute_coverage(self):
        # Generate the coverage pattern
        self.goals = self.generate_coverage_pattern()
        rospy.loginfo(f"[{self.namespace}] Generated coverage pattern with {len(self.goals)} waypoints")
        
        rate = rospy.Rate(2)  # Check at 2Hz for more responsive behavior
        
        while not rospy.is_shutdown():
            # If we have goals to process
            if self.current_goal_index < len(self.goals):
                # Check if we're actively pursuing a goal
                goal_state = self.action_client.get_state()
                
                if goal_state not in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
                    # If no active goal or previous goal finished
                    x, y = self.goals[self.current_goal_index]
                    self.send_goal(x, y)
                    
                # Check for failed goals
                if goal_state == actionlib.GoalStatus.ABORTED:
                    rospy.logwarn(f"[{self.namespace}] Failed to reach goal {self.current_goal_index+1}, possible obstacle")
                    
                    # Try again a couple of times if needed
                    if self.retries < self.max_retries:
                        self.retries += 1
                        rospy.loginfo(f"[{self.namespace}] Retry {self.retries}/{self.max_retries}")
                        self.send_goal(*self.goals[self.current_goal_index])
                    else:
                        # If max retries reached, skip this point
                        rospy.logwarn(f"[{self.namespace}] Skipping goal {self.current_goal_index+1} after {self.max_retries} failed attempts")
                        self.current_goal_index += 1
                        self.retries = 0
                        
                elif goal_state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo(f"[{self.namespace}] Goal {self.current_goal_index+1} reached")
                    self.current_goal_index += 1
                    self.retries = 0
                    
            else:
                # If we've completed all goals, restart
                rospy.loginfo(f"[{self.namespace}] Completed coverage pattern, restarting...")
                self.current_goal_index = 0
                
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('robot_smooth_explorer', anonymous=True)
        robot_name = rospy.get_param("~robot_name", "robot_1")
        line_spacing = rospy.get_param("~line_spacing", 1.0)
        
        if robot_name == "robot_1":
            r = SmoothLineCoverageExplorer("robot_1", [-9, 9], [3, 9], line_spacing)
        elif robot_name == "robot_2":
            r = SmoothLineCoverageExplorer("robot_2", [-9, 0], [-9, -2], line_spacing)
        elif robot_name == "robot_3":
            r = SmoothLineCoverageExplorer("robot_3", [0, 9], [-9, -2], line_spacing)
        else:
            rospy.logerr(f"Invalid robot_name: {robot_name}")
            exit(1)
            
        r.execute_coverage()
    except rospy.ROSInterruptException:
        pass