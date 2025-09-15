#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class OdometryTester(Node):
    def __init__(self):
        super().__init__('odom_tester')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # Test patterns
        self.timer = self.create_timer(0.1, self.test_sequence)
        self.start_time = time.time()
        
    def odom_cb(self, msg):
        # Log odometry for validation
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2 * math.atan2(qz, qw)
        
        self.get_logger().info(f'Odom: x={x:.3f}, y={y:.3f}, θ={yaw:.3f}')
        
    def test_sequence(self):
        elapsed = time.time() - self.start_time
        cmd = Twist()
        
        if elapsed < 5.0:  # Forward motion
            cmd.linear.x = 0.5
        elif elapsed < 10.0:  # Rotation
            cmd.angular.z = 0.5
        elif elapsed < 15.0:  # Circle
            cmd.linear.x = 0.3
            cmd.angular.z = 0.3
        else:  # Stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = OdometryTester()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()