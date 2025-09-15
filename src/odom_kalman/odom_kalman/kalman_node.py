#!/usr/bin/env python3
"""
odom_kalman: Subscribe to /joint_states, simulate encoder ticks for 4 wheels,
convert to ticks (debounced), fuse with a fixed-point Kalman filter and
publish nav_msgs/Odometry at 50 Hz.

Fixed-point implementation uses SCALE = 1000 (i.e., 3 decimal digits).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose, PoseWithCovariance, Point
from builtin_interfaces.msg import Time
import math
import time
import threading
from collections import deque

# ---------- Parameters ----------
WHEEL_RADIUS = 0.06        # meters
TICKS_PER_REV = 500        # encoder resolution simulated
SCALE = 1000               # fixed-point scale (1 -> 1000)
PUBLISH_HZ = 50
DT = 1.0 / PUBLISH_HZ

# State vector: [x, y, theta, v_lin, v_ang] in scaled integers
# For simplicity, process uses small KF: estimate pose & velocities.

class KalmanNode(Node):
    def __init__(self):
        super().__init__('odom_kalman')
        self.declare_parameter('wheel_radius', WHEEL_RADIUS)
        self.declare_parameter('ticks_per_rev', TICKS_PER_REV)

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').get_parameter_value().integer_value

        # Subscribe to joint_states
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        # Publisher: odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Internal tick counters for 4 wheels: names in URDF wheel_fl_link, wheel_fr_link, wheel_rl_link, wheel_rr_link
        self.last_positions = {'wheel_fl_link': None, 'wheel_fr_link': None, 'wheel_rl_link': None, 'wheel_rr_link': None}
        self.tick_counts = {'wheel_fl': 0, 'wheel_fr': 0, 'wheel_rl': 0, 'wheel_rr': 0}

        # Debounce: minimal delta rad to consider a tick (avoid duplicates)
        self.min_tick_rad = (2.0 * math.pi) / self.ticks_per_rev / 2.0

        # Kalman fixed-point state and covariance (scaled)
        # x = [x, y, theta, v, w] scaled by SCALE
        self.state = [0, 0, 0, 0, 0]  # integers
        # covariance P as diagonal for simplicity (scaled)
        self.P = [int(1*SCALE), int(1*SCALE), int(100*SCALE), int(1*SCALE), int(1*SCALE)]

        # Process noise Q (small)
        self.Q = [int(0.01*SCALE), int(0.01*SCALE), int(0.1*SCALE), int(0.1*SCALE), int(0.1*SCALE)]

        # Measurement noise R (we measure linear & angular velocity derived from wheels)
        self.R = [int(0.5*SCALE), int(0.5*SCALE)]  # v_lin, v_ang

        # For timing
        self.last_time = self.get_clock().now()

        # Timer to publish at 50 Hz
        self.timer = self.create_timer(DT, self.timer_cb)

        self.get_logger().info('Kalman odom node started. Publishing odom @ %d Hz' % PUBLISH_HZ)

        # Simulate GPIO interrupt counters (thread-safe)
        self.interrupt_lock = threading.Lock()
        self.hall_counters = {'fl': 0, 'fr': 0, 'rl': 0, 'rr': 0}

        # Debouncing simulation
        self.last_interrupt_time = {'fl': 0, 'fr': 0, 'rl': 0, 'rr': 0}
        self.debounce_time = 0.001  # 1ms debounce

        # Interrupt simulation buffer
        self.interrupt_buffer = deque(maxlen=1000)

    def joint_cb(self, msg: JointState):
        # Update positions into last_positions; if last exists, compute delta and convert to ticks.
        for name, pos in zip(msg.name, msg.position):
            if name in self.last_positions:
                last = self.last_positions[name]
                if last is None:
                    self.last_positions[name] = pos
                    continue
                # compute delta angle (handle wrap)
                delta = pos - last
                # normalize to [-pi, pi]
                while delta > math.pi:
                    delta -= 2*math.pi
                while delta < -math.pi:
                    delta += 2*math.pi
                # detect ticks
                wheel_key = None
                if name == 'wheel_fl_link':
                    wheel_key = 'wheel_fl'
                elif name == 'wheel_fr_link':
                    wheel_key = 'wheel_fr'
                elif name == 'wheel_rl_link':
                    wheel_key = 'wheel_rl'
                elif name == 'wheel_rr_link':
                    wheel_key = 'wheel_rr'
                if wheel_key:
                    # For each full tick rotation step, increment tick_counts
                    # Convert delta to ticks
                    ticks_delta = int(abs(delta) / ((2.0 * math.pi) / self.ticks_per_rev))
                    # Debounce: accept ticks only if delta >= min_tick_rad
                    if abs(delta) >= self.min_tick_rad:
                        self.tick_counts[wheel_key] += ticks_delta
                self.last_positions[name] = pos
            else:
                # not tracked
                pass

    def simulate_gpio_interrupt(self, wheel_id, tick_count):
        """Simulate GPIO interrupt from Hall sensor"""
        current_time = time.time()
        
        # Debouncing check
        if current_time - self.last_interrupt_time[wheel_id] < self.debounce_time:
            return
        
        with self.interrupt_lock:
            self.hall_counters[wheel_id] += tick_count
            self.last_interrupt_time[wheel_id] = current_time
            
        # Log interrupt (simulate firmware logging)
        self.get_logger().debug(f'Hall interrupt: {wheel_id}, count: {tick_count}')
    
    def timer_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9 if self.last_time is not None else DT
        if dt <= 0.0:
            dt = DT
        self.last_time = now

        # Convert ticks to linear & angular velocity
        # Assume left wheels: fl, rl; right wheels: fr, rr
        left_ticks = self.tick_counts['wheel_fl'] + self.tick_counts['wheel_rl']
        right_ticks = self.tick_counts['wheel_fr'] + self.tick_counts['wheel_rr']
        # reset after reading (like firmware might)
        self.tick_counts = {k: 0 for k in self.tick_counts}

        # Convert ticks to revolutions per second (rps)
        left_revs = left_ticks / (2.0 * self.ticks_per_rev) / dt  # two wheels sum -> average; but dividing by 2 later
        right_revs = right_ticks / (2.0 * self.ticks_per_rev) / dt

        # linear velocities (m/s)
        v_left = left_revs * 2.0 * math.pi * self.wheel_radius
        v_right = right_revs * 2.0 * math.pi * self.wheel_radius

        v_lin = (v_left + v_right) / 2.0
        # approximate angular velocity around z: (v_right - v_left) / track_width
        track_width = 2.0 * 0.25  # approx from URDF wheel_sep_y * 2
        v_ang = (v_right - v_left) / track_width

        # Convert to fixed-point integers
        z_v = int(v_lin * SCALE)
        z_w = int(v_ang * SCALE)

        # ---------- Kalman predict & update (very small, simplified KF) ----------
        # Predict step: simple motion model x' = x + v * cos(theta) * dt
        # Note: state scaled by SCALE: to compute v*dt in same units:
        # v_scaled * dt => (v*SCALE) * dt ; multiply then divide SCALE when needed

        # Extract current scaled state
        sx, sy, stheta, sv, sw = self.state  # all scaled

        # Predict pose with integer math: new_x = sx + (sv * cos(theta) * dt_scaled)
        # But cos(theta) uses floating; we keep cos(theta) in float and convert.
        theta_float = stheta / SCALE
        cos_t = math.cos(theta_float)
        sin_t = math.sin(theta_float)

        # predicted pose (floating intermediary)
        pred_x = (sx / SCALE) + (sv / SCALE) * cos_t * dt
        pred_y = (sy / SCALE) + (sv / SCALE) * sin_t * dt
        pred_theta = (stheta / SCALE) + (sw / SCALE) * dt

        # To scaled integers
        pred_x_i = int(pred_x * SCALE)
        pred_y_i = int(pred_y * SCALE)
        pred_theta_i = int(pred_theta * SCALE)
        pred_v_i = sv  # assume previous v
        pred_w_i = sw

        # Very simple covariance predict (add process noise)
        P_pred = [p + q for p, q in zip(self.P, self.Q)]

        # Measurement: we measure v_lin and v_ang (z_v, z_w)
        # Build Kalman gain for velocity states only (very simplified scalar gains)
        # K_v = P_v / (P_v + R_v)
        Pv = P_pred[3]
        Pw = P_pred[4]
        Kv = int((Pv * SCALE) / (Pv + self.R[0])) if (Pv + self.R[0]) != 0 else 0
        Kw = int((Pw * SCALE) / (Pw + self.R[1])) if (Pw + self.R[1]) != 0 else 0

        # Update velocities (scaled)
        # v_new = v_pred + K * (z - v_pred)
        # K scaled by SCALE, so careful: v_new = v_pred + (Kv/SCALE) * (z - v_pred)
        v_new_i = pred_v_i + int((Kv * (z_v - pred_v_i)) / SCALE)
        w_new_i = pred_w_i + int((Kw * (z_w - pred_w_i)) / SCALE)

        # Update P (very naive)
        Pv_new = int((SCALE - Kv) * Pv / SCALE)
        Pw_new = int((SCALE - Kw) * Pw / SCALE)
        # set back predicted covariance for pose unchanged except velocities
        self.P = [P_pred[0], P_pred[1], P_pred[2], Pv_new, Pw_new]

        # Final state
        self.state = [pred_x_i, pred_y_i, pred_theta_i, v_new_i, w_new_i]

        # Publish odometry (float messages)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.state[0] / SCALE
        odom.pose.pose.position.y = self.state[1] / SCALE
        odom.pose.pose.position.z = 0.0
        # convert theta to quaternion (yaw)
        yaw = self.state[2] / SCALE
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.state[3] / SCALE
        odom.twist.twist.angular.z = self.state[4] / SCALE

        # naive covariances (fill partial)
        odom.pose.covariance[0] = float(self.P[0]) / SCALE
        odom.pose.covariance[7] = float(self.P[1]) / SCALE
        odom.pose.covariance[35] = float(self.P[2]) / SCALE

        # Publish
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
