#!/usr/bin/env python3
import rclpy, time, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class CmdVelGuard(Node):
    def __init__(self):
        super().__init__('cmd_vel_guard')
        self.declare_parameter('front_arc_deg', 60.0)
        self.declare_parameter('slow_dist', 0.60)
        self.declare_parameter('stop_dist', 0.35)
        self.declare_parameter('alpha', 0.3)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('watchdog_sec', 0.5)
        self.declare_parameter('source_cmd', '/cmd_vel_nav')
        self.front_arc = math.radians(self.get_parameter('front_arc_deg').value)
        self.slow_dist = float(self.get_parameter('slow_dist').value)
        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.alpha     = float(self.get_parameter('alpha').value)
        self.scan_topic= self.get_parameter('scan_topic').value
        self.watchdog  = float(self.get_parameter('watchdog_sec').value)
        self.source    = self.get_parameter('source_cmd').value
        self.sub_cmd   = self.create_subscription(Twist, self.source, self.on_cmd, 10)
        self.sub_scan  = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.pub_cmd   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer     = self.create_timer(0.05, self.on_timer)
        self.last_scan_t = 0.0
        self.min_front   = float('inf')
        self.vx_f = 0.0
        self.wz_f = 0.0
        self.pending     = Twist()

    def on_scan(self, msg: LaserScan):
        self.last_scan_t = time.time()
        angles = [msg.angle_min + i*msg.angle_increment for i in range(len(msg.ranges))]
        front = []
        half = self.front_arc/2.0
        for a, r in zip(angles, msg.ranges):
            if -half <= a <= half and math.isfinite(r) and r > 0.0:
                front.append(r)
        self.min_front = min(front) if front else float('inf')

    def on_cmd(self, msg: Twist):
        self.pending = msg

    def on_timer(self):
        out = Twist()
        if time.time() - self.last_scan_t > self.watchdog:
            self.pub_cmd.publish(out)  # stop if scan stale
            return

        vx = self.pending.linear.x
        wz = self.pending.angular.z

        if self.min_front <= self.stop_dist:
            vx = 0.0
        elif self.min_front <= self.slow_dist:
            k = (self.min_front - self.stop_dist) / max(1e-3, (self.slow_dist - self.stop_dist))
            vx = vx * max(0.0, min(1.0, k))

        self.vx_f = self.alpha * vx + (1.0 - self.alpha) * self.vx_f
        self.wz_f = self.alpha * wz + (1.0 - self.alpha) * self.wz_f

        out.linear.x  = float(self.vx_f)
        out.angular.z = float(self.wz_f)
        self.pub_cmd.publish(out)

def main():
    rclpy.init()
    n = CmdVelGuard()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
