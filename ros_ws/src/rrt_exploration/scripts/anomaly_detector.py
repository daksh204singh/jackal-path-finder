#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class AnomalyDetectorNode:
    def __init__(self):
        rospy.init_node('anomaly_detector')
        self.subscription = rospy.Subscriber(
            'robot_1/front/image_raw',
            Image,
            self.image_callback,
            queue_size=10)
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher('/anomaly_detection/image', Image, queue_size=10)
        
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.add(mask1, mask2)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            
            if area < 100:
                continue
                
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            if circularity > 0.7:
                x, y, w, h = cv2.boundingRect(contour)
                
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                center_x = x + w // 2
                center_y = y + h // 2
                
                cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
                
                cv2.putText(cv_image, f"({center_x}, {center_y})", 
                           (center_x - 50, center_y - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        self.publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

def main():
    anomaly_detector = AnomalyDetectorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
