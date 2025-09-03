#!/usr/bin/env python3
import argparse, rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

class MagScaler(Node):
    def __init__(self, in_topic, out_topic, scale):
        super().__init__('mag_scale')
        self.scale = float(scale)
        self.sub = self.create_subscription(MagneticField, in_topic, self.cb, 10)
        self.pub = self.create_publisher(MagneticField, out_topic, 10)
        self.get_logger().info(f"Scaling {in_topic} -> {out_topic} by {self.scale}")

    def cb(self, m: MagneticField):
        k = self.scale
        k2 = k * k
        out = MagneticField()
        out.header = m.header
        out.magnetic_field.x = m.magnetic_field.x * k
        out.magnetic_field.y = m.magnetic_field.y * k
        out.magnetic_field.z = m.magnetic_field.z * k
        cov = list(m.magnetic_field_covariance)
        if any(c != 0.0 for c in cov):
            out.magnetic_field_covariance = [c * k2 for c in cov]
        else:
            out.magnetic_field_covariance = cov
        self.pub.publish(out)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in',  dest='in_topic',  default='/imu/mag')
    ap.add_argument('--out', dest='out_topic', default='/imu/mag_tesla')
    ap.add_argument('--scale', type=float, required=True)
    args = ap.parse_args()
    rclpy.init()
    rclpy.spin(MagScaler(args.in_topic, args.out_topic, args.scale))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
