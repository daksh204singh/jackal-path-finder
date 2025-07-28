#!/usr/bin/env python3
import rospy
import random
import cv2
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool

class BallDetectionAndCoordination:
    def __init__(self, namespace, x_range, y_range, min_distance=1.0):
        self.namespace = namespace
        self.x_range = x_range
        self.y_range = y_range
        self.min_distance = min_distance
        self.ball_radius = 0.5  # Expected ball radius in meters
        
        # State variables
        self.current_pose = None
        self.ball_found = False
        self.ball_world_pos = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = None
        self.robot_id = int(namespace.split('_')[-1]) if namespace.split('_')[-1].isdigit() else 0

        # Set up CV bridge
        self.bridge = CvBridge()
        
        # Set up TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Define frames
        self.base_frame = f"{namespace}/base_link"
        self.map_frame = "map"
        
        # Publishers
        self.goal_pub = rospy.Publisher(f'/{namespace}/move_base_simple/goal', PoseStamped, queue_size=1)
        self.ball_pub = rospy.Publisher('/ball_position', Point, queue_size=10)
        self.ball_detection_pub = rospy.Publisher(f'/{namespace}/ball_detected', Bool, queue_size=10)
        self.debug_image_pub = rospy.Publisher(f'/{namespace}/detection/image', Image, queue_size=10)
        
        # Subscribers
        rospy.Subscriber(f'/{namespace}/odom', Odometry, self.odom_callback)
        rospy.Subscriber(f'/{namespace}/front/image_raw', Image, self.image_callback)
        rospy.Subscriber(f'/{namespace}/front/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/ball_position', Point, self.ball_found_callback)

        rospy.loginfo(f"[{self.namespace}] Initialized with exploration range X: {x_range}, Y: {y_range}")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def camera_info_callback(self, data):
        """Receive camera intrinsics."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(data.K).reshape(3,3)
            self.dist_coeffs = np.array(data.D)
            fid = data.header.frame_id or f"{self.namespace}/camera_link"
            self.camera_frame = fid
            rospy.loginfo(f"[{self.namespace}] Camera calibrated. frame={self.camera_frame}")

    def image_callback(self, data):
        if self.ball_found or self.camera_matrix is None:
            return
        
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Convert to HSV for better color segmentation
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define color range for ball detection (assuming a red ball)
            # Adjust these values based on the actual color of your ball
            lower_color = np.array([0, 100, 100])
            upper_color = np.array([10, 255, 255])
            
            # Create a mask
            mask = cv2.inRange(hsv, lower_color, upper_color)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                # Filter small contours
                if cv2.contourArea(contour) < 100:
                    continue
                
                # Check circularity
                perimeter = cv2.arcLength(contour, True)
                if perimeter == 0:
                    continue
                
                area = cv2.contourArea(contour)
                circularity = 4 * math.pi * area / (perimeter * perimeter)
                
                if circularity > 0.7:  # Close to a circle
                    # Find enclosing circle
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    
                    # Draw the circle on the debug image
                    cv2.circle(cv_image, center, radius, (0, 255, 0), 2)
                    cv2.putText(cv_image, f"Ball detected!", (center[0] - 20, center[1] - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Calculate 3D position
                    if self.detect_ball_3d(center, radius):
                        # Mark detection
                        cv2.putText(cv_image, "Ball localized in 3D!", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Add searching text if no ball found
            if not self.ball_found:
                cv2.putText(cv_image, "Searching for ball...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Publish debug image
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                
        except CvBridgeError as e:
            rospy.logerr(f"[{self.namespace}] CVBridge error: {e}")

    def detect_ball_3d(self, center, radius):
        """Estimate 3D position of ball from image coordinates."""
        if self.camera_matrix is None or self.camera_frame is None:
            return False
            
        # Calculate distance based on apparent radius
        # Simplified calculation: distance = (real_radius * focal_length) / apparent_radius
        focal_length = self.camera_matrix[0, 0]  # Assuming fx ≈ fy
        estimated_distance = (self.ball_radius * focal_length) / radius
        
        # Calculate 3D ray from camera
        x = (center[0] - self.camera_matrix[0, 2]) / focal_length
        y = (center[1] - self.camera_matrix[1, 2]) / focal_length
        
        # Normalize direction vector
        norm = math.sqrt(x*x + y*y + 1)
        x /= norm
        y /= norm
        z = 1.0 / norm
        
        # Calculate 3D point in camera frame
        point_camera = np.array([x, y, z]) * estimated_distance
        
        try:
            # Transform point from camera to map frame
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            # Apply transform to get ball position in world coordinates
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            # Extract rotation from transform
            q = transform.transform.rotation
            rotation = np.array([q.x, q.y, q.z, q.w])
            rot_matrix = self.quaternion_to_rotation_matrix(rotation)
            
            # Apply rotation and translation
            ball_world = translation + np.dot(rot_matrix, point_camera)
            
            # Publish ball position
            ball_msg = Point()
            ball_msg.x = ball_world[0]
            ball_msg.y = ball_world[1]
            ball_msg.z = ball_world[2]
            
            self.ball_pub.publish(ball_msg)
            self.ball_detection_pub.publish(Bool(True))
            
            rospy.loginfo(f"[{self.namespace}] Ball detected at world position: ({ball_world[0]:.2f}, {ball_world[1]:.2f}, {ball_world[2]:.2f})")
            
            # Store ball position and set flag
            self.ball_world_pos = ball_world
            self.ball_found = True
            
            # Move towards the ball
            self.move_towards_ball()
            
            return True
            
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"[{self.namespace}] TF error when detecting ball: {e}")
            return False

    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix."""
        x, y, z, w = q
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])

    def ball_found_callback(self, msg):
        """Called when any robot detects the ball."""
        if self.ball_found:
            return
            
        rospy.loginfo(f"[{self.namespace}] Ball detected by another robot at ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}).")
        self.ball_found = True
        self.ball_world_pos = np.array([msg.x, msg.y, msg.z])
        self.move_towards_ball()

    def move_towards_ball(self):
        """Move to surround the ball in a circular formation."""
        if self.current_pose is None or self.ball_world_pos is None:
            rospy.logwarn(f"[{self.namespace}] Cannot approach ball: missing pose or ball position.")
            return

        try:
            # Get current robot position in map frame
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            rx, ry = tf.transform.translation.x, tf.transform.translation.y
            
            # Calculate direction to ball
            dx = self.ball_world_pos[0] - rx
            dy = self.ball_world_pos[1] - ry
            angle_to_ball = math.atan2(dy, dx)
            
            # Calculate surrounding position
            num_robots = 3  # Total number of robots
            surround_radius = self.ball_radius + 1.5  # Distance from ball center to robots (ball radius + buffer)
            
            # Calculate angle offset based on robot ID
            offset_angle = angle_to_ball + math.pi + 2*math.pi*self.robot_id/num_robots
            
            # Calculate target position around the ball
            tx = self.ball_world_pos[0] + surround_radius * math.cos(offset_angle)
            ty = self.ball_world_pos[1] + surround_radius * math.sin(offset_angle)
            
            # Create and send goal
            goal = PoseStamped()
            goal.header.frame_id = self.map_frame
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = tx
            goal.pose.position.y = ty
            
            # Calculate orientation to face the ball
            face_angle = math.atan2(self.ball_world_pos[1] - ty, self.ball_world_pos[0] - tx)
            q = quaternion_from_euler(0, 0, face_angle)
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]
            
            self.goal_pub.publish(goal)
            rospy.loginfo(f"[{self.namespace}] Moving to surround position ({tx:.2f}, {ty:.2f}), facing ball.")
            
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"[{self.namespace}] TF error when surrounding ball: {e}")

    def generate_random_goal(self):
        """Generate a random exploration goal."""
        if self.ball_found:
            return None, None  # Don't generate random goals if ball is found
            
        x = random.uniform(self.x_range[0], self.x_range[1])
        y = random.uniform(self.y_range[0], self.y_range[1])
        
        if self.current_pose:
            dx = x - self.current_pose.position.x
            dy = y - self.current_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
            
            # Ensure goal is far enough from current position
            if distance < self.min_distance:
                return self.generate_random_goal()
        
        return x, y

    def explore(self):
        """Main function to control the robot's exploration."""
        rospy.init_node(f'{self.namespace}_ball_detection_coordination')
        rate = rospy.Rate(0.05)  # One goal every 20s
        
        while not rospy.is_shutdown():
            if not self.ball_found:
                x, y = self.generate_random_goal()
                if x is not None and y is not None:
                    goal = PoseStamped()
                    goal.header.frame_id = self.map_frame
                    goal.header.stamp = rospy.Time.now()
                    goal.pose.position.x = x
                    goal.pose.position.y = y
                    goal.pose.orientation.w = 1.0
                    
                    self.goal_pub.publish(goal)
                    rospy.loginfo(f"[{self.namespace}] Exploring: sent goal to x={x:.2f}, y={y:.2f}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        robot_name = rospy.get_param("~robot_name", "robot_1")
        
        if robot_name == "robot_1":
            r = BallDetectionAndCoordination("robot_1", [-9, 9], [3, 9])
        elif robot_name == "robot_2":
            r = BallDetectionAndCoordination("robot_2", [-9, 0], [-9, -2])
        elif robot_name == "robot_3":
            r = BallDetectionAndCoordination("robot_3", [0, 9], [-9, -2])
        else:
            rospy.logerr(f"Unknown robot name: {robot_name}")
            exit(1)
            
        r.explore()
    except rospy.ROSInterruptException:
        pass