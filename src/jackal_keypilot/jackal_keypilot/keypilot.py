#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time
import sys

# Using pynput for reliable keyboard input detection
from pynput import keyboard

class JackalGameController(Node):
    def __init__(self):
        super().__init__('jackal_game_controller')
        
        # Create a publisher for the cmd_vel topic
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Current velocity values
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Target velocity values (what we're trying to reach)
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        
        # Maximum velocity values for safety
        self.max_linear_velocity = 0.5  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        
        # Acceleration and deceleration rates (units per second)
        self.linear_acceleration = 1.0  # m/s²
        self.linear_deceleration = 0.8  # m/s²
        self.angular_acceleration = 2.0  # rad/s²
        self.angular_deceleration = 1.5  # rad/s²
        
        # Set up key state tracking
        self.key_states = {
            'w': False,
            'a': False,
            's': False,
            'd': False
        }
        
        # Flag for running
        self.running = True
        
        # Set up keyboard listener
        self.keyboard_listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.keyboard_listener.start()
        
        # Create a timer for publishing velocity commands (50Hz for smooth interpolation)
        self.update_rate = 0.02  # 50Hz
        self.timer = self.create_timer(self.update_rate, self.update_and_publish_velocity)
        
        # Print instructions
        self.print_instructions()
    
    def print_instructions(self):
        self.get_logger().info('Jackal Videogame-Style Control:')
        self.get_logger().info('------------------------------')
        self.get_logger().info('W: Move forward')
        self.get_logger().info('S: Move backward')
        self.get_logger().info('A: Turn left')
        self.get_logger().info('D: Turn right')
        self.get_logger().info('Q: Quit')
        self.get_logger().info('------------------------------')
        self.get_logger().info('Game-like smooth acceleration and deceleration')
    
    def on_press(self, key):
        """Handle key press events"""
        try:
            key_char = key.char.lower()
            if key_char in self.key_states:
                self.key_states[key_char] = True
            elif key_char == 'q':
                self.get_logger().info('Quitting...')
                self.running = False
                self.keyboard_listener.stop()
                # Send a stop command before shutting down
                self.emergency_stop()
                rclpy.shutdown()
                sys.exit(0)
        except (AttributeError, TypeError):
            # Special keys not handled
            pass
    
    def on_release(self, key):
        """Handle key release events"""
        try:
            key_char = key.char.lower()
            if key_char in self.key_states:
                self.key_states[key_char] = False
        except (AttributeError, TypeError):
            # Special keys not handled
            pass
    
    def update_target_velocities(self):
        """Update target velocities based on key states"""
        # Set target linear velocity based on W/S keys
        if self.key_states['w'] and not self.key_states['s']:
            self.target_linear_velocity = self.max_linear_velocity
        elif self.key_states['s'] and not self.key_states['w']:
            self.target_linear_velocity = -self.max_linear_velocity
        else:
            self.target_linear_velocity = 0.0
        
        # Set target angular velocity based on A/D keys
        if self.key_states['a'] and not self.key_states['d']:
            self.target_angular_velocity = self.max_angular_velocity
        elif self.key_states['d'] and not self.key_states['a']:
            self.target_angular_velocity = -self.max_angular_velocity
        else:
            self.target_angular_velocity = 0.0
    
    def apply_smooth_acceleration(self):
        """Gradually adjust current velocities toward target velocities"""
        # Calculate maximum change in this update cycle
        linear_max_change = (self.linear_acceleration if abs(self.target_linear_velocity) > abs(self.linear_velocity) 
                            else self.linear_deceleration) * self.update_rate
        
        angular_max_change = (self.angular_acceleration if abs(self.target_angular_velocity) > abs(self.angular_velocity)
                             else self.angular_deceleration) * self.update_rate
        
        # Linear velocity smoothing
        if abs(self.target_linear_velocity - self.linear_velocity) <= linear_max_change:
            # Close enough to just set it directly
            self.linear_velocity = self.target_linear_velocity
        else:
            # Move toward target at the maximum allowed rate
            if self.linear_velocity < self.target_linear_velocity:
                self.linear_velocity += linear_max_change
            else:
                self.linear_velocity -= linear_max_change
        
        # Angular velocity smoothing
        if abs(self.target_angular_velocity - self.angular_velocity) <= angular_max_change:
            # Close enough to just set it directly
            self.angular_velocity = self.target_angular_velocity
        else:
            # Move toward target at the maximum allowed rate
            if self.angular_velocity < self.target_angular_velocity:
                self.angular_velocity += angular_max_change
            else:
                self.angular_velocity -= angular_max_change
    
    def update_and_publish_velocity(self):
        """Update velocities and publish to ROS"""
        # Update target velocities based on key states
        self.update_target_velocities()
        
        # Apply smooth acceleration/deceleration
        self.apply_smooth_acceleration()
        
        # Create and publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = self.angular_velocity
        
        self.publisher.publish(twist_msg)
        
        # Debug output if movement is happening
        if abs(self.linear_velocity) > 0.01 or abs(self.angular_velocity) > 0.01:
            self.get_logger().debug(f'Linear: {self.linear_velocity:.2f}, Angular: {self.angular_velocity:.2f}')
    
    def emergency_stop(self):
        """Send an immediate stop command"""
        twist_msg = Twist()
        # All velocities set to zero
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        
        # Publish several times to ensure it's received
        for _ in range(5):
            self.publisher.publish(twist_msg)
            time.sleep(0.01)
    
    def __del__(self):
        """Clean up when the node is destroyed"""
        self.running = False
        if hasattr(self, 'keyboard_listener'):
            self.keyboard_listener.stop()
        
        # Try to stop the robot before shutting down
        try:
            self.emergency_stop()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    controller = JackalGameController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.running = False
        if hasattr(controller, 'keyboard_listener'):
            controller.keyboard_listener.stop()
        
        # Try to stop the robot before shutting down
        try:
            controller.emergency_stop()
        except:
            pass
            
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()