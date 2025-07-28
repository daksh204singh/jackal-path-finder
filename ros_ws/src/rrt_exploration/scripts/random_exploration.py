#!/usr/bin/env python3
import rospy
import random
import math
import time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class RobotCoordinator:
    def __init__(self, namespace, x_range, y_range, min_distance=1.0):
        self.namespace = namespace
        self.x_range = x_range
        self.y_range = y_range
        self.min_distance = min_distance
        self.current_pose = None
        self.goal_reached = True # Start as True to trigger sending the first goal
        self.current_goal_x = None
        self.current_goal_y = None
        self.goal_tolerance = 0.5  # How close robot needs to get to goal
        self.start_time = None
        self.startup_delay = 30.0  # Wait 30 seconds before starting
        self.has_started = False # Flag to indicate startup delay passed

        # Use a more descriptive name if possible, or ensure node names are unique
        try:
            rospy.init_node(f'{self.namespace}_coordinator', anonymous=False)
            rospy.loginfo(f"Initializing {self.namespace}_coordinator node")
        except rospy.exceptions.ROSException as e:
            rospy.logerr(f"Error initializing node for {self.namespace}: {e}. Namespace might be in use.")
            # Handle error appropriately, maybe exit or retry
            return


        # Publishers
        self.pub = rospy.Publisher(f'/{namespace}/move_base_simple/goal', PoseStamped, queue_size=1)

        # Subscribers
        rospy.Subscriber(f'/{namespace}/odom', Odometry, self.odom_callback)

        # Specific setups for each robot type
        if namespace == "robot_1":
            self.target_pub = rospy.Publisher('/robot_1_at_target', Bool, queue_size=1, latch=True) # Latch the final message
            self.targets = [(7.0, 7.0), (-7.0, 7.0)]  # Predefined targets for Robot 1
            self.current_target_idx = 0
            self.robot1_at_final_target_published = False # Flag to ensure message published only once
        else:
            # Robots 2 and 3 subscribe to robot_1's status
            rospy.Subscriber('/robot_1_at_target', Bool, self.robot1_status_callback)
            self.robot1_at_final_target = False
            self.final_formation_goal_sent = False # Track if the final goal was sent

    def robot1_status_callback(self, msg):
        # Only update if the status changes to True
        if msg.data and not self.robot1_at_final_target:
             self.robot1_at_final_target = True
             # Crucially, force goal_reached to True to trigger sending the new surrounding goal immediately
             self.goal_reached = True
             # Cancel the current random goal if move_base supports it (optional but cleaner)
             # self.cancel_current_goal() # Requires actionlib client, more complex setup
             rospy.loginfo(f"[{self.namespace}] Received update: Robot 1 at final target. Switching to formation goal.")

    def odom_callbafck(self, msg):
        self.current_pose = msg.pose.pose
        if not self.has_started or self.goal_reached or self.current_goal_x is None:
            # Don't check for goal achievement if we haven't started,
            # have already reached the goal, or haven't sent one yet.
            return

        # Check if the current goal has been reached
        dx = self.current_goal_x - self.current_pose.position.x
        dy = self.current_goal_y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.goal_tolerance:
            rospy.loginfo(f"[{self.namespace}] Reached target ({self.current_goal_x:.2f}, {self.current_goal_y:.2f})")
            self.goal_reached = True

            # Specific logic for Robot 1 after reaching a target
            if self.namespace == "robot_1":
                # Check if it was the final target that was just reached
                if self.current_target_idx == len(self.targets) - 1 and not self.robot1_at_final_target_published:
                    rospy.loginfo(f"[{self.namespace}] Reached FINAL target {self.current_target_idx + 1}.")
                    msg = Bool()
                    msg.data = True
                    self.target_pub.publish(msg)
                    self.robot1_at_final_target_published = True # Prevent re-publishing
                    rospy.loginfo("Robot 1 reached final target! Signaling other robots.")
                # Advance to the next target index (even if it's out of bounds, run() checks)
                self.current_target_idx += 1


    def generate_random_goal(self):
        if not self.current_pose: # Should ideally not happen after odom starts
             rospy.logwarn(f"[{self.namespace}] Cannot generate relative goal, current_pose not yet received. Generating absolute random.")
             return random.uniform(self.x_range[0], self.x_range[1]), random.uniform(self.y_range[0], self.y_range[1])

        # Try to generate a goal that's at least min_distance away
        max_attempts = 10
        for _ in range(max_attempts):
            x = random.uniform(self.x_range[0], self.x_range[1])
            y = random.uniform(self.y_range[0], self.y_range[1])

            dx = x - self.current_pose.position.x
            dy = y - self.current_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)

            if distance >= self.min_distance:
                return x, y

        # If we couldn't find a suitable point after attempts, just return a random one
        rospy.logdebug(f"[{self.namespace}] Could not find goal > min_distance away after {max_attempts} attempts. Using last random.")
        return x, y # Return the last generated point even if too close


    def generate_surrounding_position(self, center_x, center_y, idx):
        # Positions around the ball at (-7, 7)
        # Robot 2 (idx=1) at 120 deg, Robot 3 (idx=2) at 240 deg relative to robot 1 (assumed at 0 deg)
        radius = 1.5  # Distance from the center point
        # Base angle needs adjustment if robot 1 isn't directly north/south/east/west of the target
        # Assuming Robot 1 (-7, 7) is the reference '0 degrees' point for the formation.
        # Let's place Robot 2 at 120 deg and Robot 3 at 240 deg relative to the positive X-axis from the center point (-7, 7).
        angle_deg = 120 * idx # Robot 2 -> 120 deg, Robot 3 -> 240 deg
        angle_rad = math.radians(angle_deg)
        x = center_x + radius * math.cos(angle_rad)
        y = center_y + radius * math.sin(angle_rad)
        return x, y

    def send_goal(self, x, y):
        # Only send if the goal is actually different from the current one,
        # unless we explicitly know we need to resend (which move_base usually handles)
        # This simple check prevents spamming the same goal if logic allows.
        # However, for the state transition (random -> surround), we need to ensure it sends.
        # The goal_reached flag handles triggering the send.

        goal = PoseStamped()
        goal.header.frame_id = "map" # Ensure this frame exists in your TF tree
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        # Set orientation to face the center point (-7, 7) for surrounding robots (optional)
        if self.namespace != "robot_1" and hasattr(self, 'robot1_at_final_target') and self.robot1_at_final_target:
            angle_to_center = math.atan2(7.0 - y, -7.0 - x)
            # Convert yaw angle to quaternion
            cy = math.cos(angle_to_center * 0.5)
            sy = math.sin(angle_to_center * 0.5)
            goal.pose.orientation.z = sy
            goal.pose.orientation.w = cy
        else:
            goal.pose.orientation.w = 1.0 # Default orientation


        self.pub.publish(goal)

        # Store current goal for checking if reached
        self.current_goal_x = x
        self.current_goal_y = y
        self.goal_reached = False # We've just sent a new goal, so it's not reached yet

        rospy.loginfo(f"[{self.namespace}] Sent goal to x={x:.2f}, y={y:.2f}")

    def run(self):
        rate = rospy.Rate(1.0)  # Loop frequency

        self.start_time = rospy.Time.now().to_sec()
        rospy.loginfo(f"[{self.namespace}] Starting coordination logic. Waiting for {self.startup_delay} second startup delay...")

        # Startup delay loop
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - self.start_time < self.startup_delay:
            # Log remaining time occasionally
            # if int(rospy.Time.now().to_sec()) % 5 == 0: # Avoid spamming log
            #     remaining = self.startup_delay - (rospy.Time.now().to_sec() - self.start_time)
            #     rospy.loginfo(f"[{self.namespace}] Waiting to start: {remaining:.1f} seconds remaining")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo(f"[{self.namespace}] Shutdown requested during startup delay.")
                return
        rospy.loginfo(f"[{self.namespace}] Startup delay complete. Beginning operation.")
        self.has_started = True
        self.goal_reached = True # Ensure first goal is sent immediately after delay

        # Main operation loop
        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.loginfo_throttle(5, f"[{self.namespace}] Waiting for current pose from odom...")
                rate.sleep()
                continue

            # --- Robot 1 Logic ---
            if self.namespace == "robot_1":
                # Check if we need to send a goal (first time, or previous goal reached)
                # and if there are still targets left in the list.
                if self.goal_reached and self.current_target_idx < len(self.targets):
                    target_x, target_y = self.targets[self.current_target_idx]
                    rospy.loginfo(f"[{self.namespace}] Moving to target {self.current_target_idx + 1}/{len(self.targets)}: ({target_x}, {target_y})")
                    self.send_goal(target_x, target_y)
                elif self.current_target_idx >= len(self.targets):
                     # Robot 1 has finished all its targets
                     rospy.loginfo_once(f"[{self.namespace}] All targets reached. Holding final position.")
                     # No more goals to send

            # --- Robots 2 & 3 Logic ---
            else:
                # Check if Robot 1 has reached its final destination
                if self.robot1_at_final_target:
                    # State: Move to final formation position
                    # Send the goal only ONCE upon entering this state, or if somehow goal_reached becomes true again (e.g., disturbance)
                    if self.goal_reached or not self.final_formation_goal_sent:
                        surround_idx = 1 if self.namespace == "robot_2" else 2
                        final_target_x, final_target_y = -7.0, 7.0 # Robot 1's final known position
                        x, y = self.generate_surrounding_position(final_target_x, final_target_y, surround_idx)

                        # Check if the new goal is significantly different from the current one
                        # This avoids resending if goal_reached was briefly true but we are already at the formation spot.
                        needs_sending = True
                        if self.current_goal_x is not None:
                            dist_sq = (x - self.current_goal_x)**2 + (y - self.current_goal_y)**2
                            if dist_sq < 0.1**2: # Don't resend if goal is basically the same
                                needs_sending = False

                        if needs_sending or not self.final_formation_goal_sent:
                             rospy.loginfo(f"[{self.namespace}] Robot 1 at target. Moving to formation point around ({final_target_x}, {final_target_y}).")
                             self.send_goal(x, y)
                             self.final_formation_goal_sent = True # Mark that we've sent the formation goal
                        else:
                            # If goal is the same and we already sent it, just ensure goal_reached is false
                            # if it was true, allowing odom_callback to monitor arrival.
                            if self.goal_reached:
                                self.goal_reached = False


                else:
                    # State: Move randomly (Robot 1 not yet at final target)
                    if self.goal_reached:
                        rospy.loginfo(f"[{self.namespace}] Reached previous random goal. Generating new random goal.")
                        x, y = self.generate_random_goal()
                        self.send_goal(x, y)

            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo(f"[{self.namespace}] Shutdown requested.")
                break

def create_coordinator(robot_name):
    # Define zones for each robot
    zones = {
        "robot_1": {"x": [-9, 9], "y": [3, 9]}, # Robot 1 moves across the top
        "robot_2": {"x": [-9, 0], "y": [-9, -2]}, # Robot 2 random lower left
        "robot_3": {"x": [0, 9], "y": [-9, -2]}  # Robot 3 random lower right
    }
    if robot_name in zones:
        rospy.loginfo(f"Creating coordinator for {robot_name} in zone X:{zones[robot_name]['x']}, Y:{zones[robot_name]['y']}")
        return RobotCoordinator(robot_name, zones[robot_name]['x'], zones[robot_name]['y'])
    else:
        rospy.logerr(f"Unknown robot name: {robot_name}")
        return None

if __name__ == '__main__':
    try:
        # Get the robot name from ROS parameter server (e.g., passed via launch file)
        # Default to robot_1 if not specified
        robot_name = rospy.get_param("~robot_name", "robot_3")
        rospy.loginfo(f"--- Starting coordinator script for robot: {robot_name} ---")

        coordinator = create_coordinator(robot_name)
        if coordinator and hasattr(coordinator, 'run'): # Check if init was successful
            coordinator.run()
        else:
            rospy.logerr(f"Failed to create or initialize coordinator for {robot_name}. Exiting.")

    except rospy.ROSInterruptException:
        rospy.loginfo("Coordinator script interrupted. Exiting.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred in the coordinator script: {e}")