#!/usr/bin/env python3
"""
Simple wheel controller that converts cmd_vel to individual wheel commands
and publishes joint_states for testing the Kalman filter.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math
import time

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        
        # Parameters
        self.wheel_radius = 0.06  # meters
        self.wheel_separation = 0.445  # meters
        
        # Publishers and subscribers  
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Wheel states (positions in radians)
        self.wheel_positions = {
            'wheel_fl_link': 0.0, 'wheel_fr_link': 0.0, 
            'wheel_rl_link': 0.0, 'wheel_rr_link': 0.0
        }
        self.wheel_velocities = {
            'wheel_fl_link': 0.0, 'wheel_fr_link': 0.0,
            'wheel_rl_link': 0.0, 'wheel_rr_link': 0.0
        }
        
        # Current command
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Timer for publishing joint states at 50Hz
        self.timer = self.create_timer(1.0/50.0, self.publish_joint_states)
        
        self.get_logger().info('Wheel controller started - listening to /cmd_vel')
        
    def cmd_callback(self, msg):
        """Convert cmd_vel to wheel velocities using differential drive kinematics"""
        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z
        
        # Differential drive kinematics
        v_left = self.current_linear - (self.current_angular * self.wheel_separation) / 2.0
        v_right = self.current_linear + (self.current_angular * self.wheel_separation) / 2.0
        
        # Convert to wheel angular velocities (rad/s)
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius
        
        # Update wheel velocities
        self.wheel_velocities['wheel_fl_link'] = omega_left
        self.wheel_velocities['wheel_rl_link'] = omega_left
        self.wheel_velocities['wheel_fr_link'] = omega_right
        self.wheel_velocities['wheel_rr_link'] = omega_right
        
        self.get_logger().debug(f'Cmd: lin={self.current_linear:.2f}, ang={self.current_angular:.2f} -> Left: {omega_left:.2f}, Right: {omega_right:.2f} rad/s')
        
    def publish_joint_states(self):
        """Publish joint states by integrating wheel velocities"""
        dt = 1.0 / 50.0  # 50Hz
        
        # Integrate velocities to get positions
        for joint in self.wheel_positions:
            self.wheel_positions[joint] += self.wheel_velocities[joint] * dt
            # Wrap angles to [-pi, pi] for realism
            while self.wheel_positions[joint] > math.pi:
                self.wheel_positions[joint] -= 2 * math.pi
            while self.wheel_positions[joint] < -math.pi:
                self.wheel_positions[joint] += 2 * math.pi
        
        # Create JointState message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        # Use the exact names from your URDF
        joint_state.name = ['wheel_fl_link', 'wheel_fr_link', 'wheel_rl_link', 'wheel_rr_link']
        joint_state.position = [
            self.wheel_positions['wheel_fl_link'],
            self.wheel_positions['wheel_fr_link'], 
            self.wheel_positions['wheel_rl_link'],
            self.wheel_positions['wheel_rr_link']
        ]
        joint_state.velocity = [
            self.wheel_velocities['wheel_fl_link'],
            self.wheel_velocities['wheel_fr_link'],
            self.wheel_velocities['wheel_rl_link'], 
            self.wheel_velocities['wheel_rr_link']
        ]
        joint_state.effort = []
        
        self.joint_pub.publish(joint_state)

def main():
    rclpy.init()
    node = WheelController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Wheel controller shutting down')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
