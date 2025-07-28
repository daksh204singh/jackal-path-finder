#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class AnomalyDetectorNode(Node):
    def __init__(self):
        super().__init__('anomaly_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/anomaly_detection/image', 10)
        
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define range for red color (note: red wraps around in HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # Create masks for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.add(mask1, mask2)
        
        # Apply morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Process contours to find circles
        for contour in contours:
            # Calculate area and perimeter
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            
            # Filter small contours
            if area < 100:
                continue
                
            # Calculate circularity
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            # If shape is approximately circular
            if circularity > 0.7:
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                
                # Draw rectangle around the ball
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Calculate center
                center_x = x + w // 2
                center_y = y + h // 2
                
                # Draw center point
                cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
                
                # Display coordinates
                cv2.putText(cv_image, f"({center_x}, {center_y})", 
                           (center_x - 50, center_y - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Publish the processed image
        self.publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    anomaly_detector = AnomalyDetectorNode()
    rclpy.spin(anomaly_detector)
    anomaly_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
