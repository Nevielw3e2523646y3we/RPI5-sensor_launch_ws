#!/usr/bin/env python3
import argparse, math, csv
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


def make_qos(reliability: str, depth: int = 100) -> QoSProfile:
    qos = QoSProfile(depth=depth)
    qos.history = HistoryPolicy.KEEP_LAST
    qos.durability = DurabilityPolicy.VOLATILE
    qos.reliability = ReliabilityPolicy.BEST_EFFORT if reliability.lower() == 'best_effort' else ReliabilityPolicy.RELIABLE
    return qos


class IMUBias(Node):
    def __init__(self, topic: str, duration_s: float, out_csv: str, reliability: str):
        super().__init__('imu_bias_live')
        self.t0 = None
        self.duration_s = duration_s
        self.times, self.gz = [], []
        self.out_csv = out_csv

        # Subscribe with requested QoS reliability (best_effort/reliable)
        self.create_subscription(Imu, topic, self.cb, make_qos(reliability))

    def cb(self, msg: Imu):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        self.times.append(t)
        self.gz.append(float(msg.angular_velocity.z))
        if t >= self.duration_s:
            self.finish()

    def finish(self):
        if len(self.times) < 5:
            print("Not enough samples; extend duration.")
            rclpy.shutdown()
            return

        ts = np.asarray(self.times, dtype=float)
        gz = np.asarray(self.gz, dtype=float)

        dts = np.diff(ts)
        med_dt = float(np.median(dts)) if dts.size else float('nan')
        fs = (1.0 / med_dt) if med_dt > 0 else float('nan')

        bias = float(np.mean(gz))
        std  = float(np.std(gz, ddof=0))
        var  = std * std

        # Stationary drift in degrees over the capture
        yaw_drift_deg = float(np.trapz(gz, ts) * 180.0 / math.pi)

        # Save CSV of raw samples
        with open(self.out_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_sec', 'gyro_z_rad_s'])
            for t, x in zip(ts, gz):
                w.writerow([f'{t:.6f}', f'{x:.9f}'])

        print("\n=== IMU yaw-rate (gyro.z) live report ===")
        print(f"Samples: {len(gz)}   median dt: {med_dt:.6f}s  (~{fs:.1f} Hz)")
        print(f"Mean (bias): {bias:.6f} rad/s")
        print(f"Std-dev:     {std:.6f} rad/s   (variance {var:.8f})")
        print(f"Stationary yaw drift over {ts[-1]:.1f}s: {yaw_drift_deg:+.3f} deg")
        print("\nApply this in your IMU publisher:")
        print(f"  imu_msg.angular_velocity.z -= {bias:.6f}")
        print("  imu_msg.angular_velocity_covariance[0] = 9999.0  # x unused")
        print("  imu_msg.angular_velocity_covariance[4] = 9999.0  # y unused")
        print(f"  imu_msg.angular_velocity_covariance[8] = {max(var,1e-5):.8f}  # z variance")
        print(f"\nCSV saved: {self.out_csv}")
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description="Live IMU yaw-rate bias/noise")
    ap.add_argument('--topic', default='/imu/data', help='IMU topic (default: /imu/data)')
    ap.add_argument('--duration', type=float, default=60.0, help='Seconds to record (default: 60)')
    ap.add_argument('--out', default='imu_bias_live.csv', help='Output CSV path')
    ap.add_argument('--reliability', choices=['best_effort', 'reliable'], default='best_effort',
                    help='Subscriber reliability QoS (default: best_effort)')
    args = ap.parse_args()

    rclpy.init()
    node = IMUBias(args.topic, args.duration, args.out, args.reliability)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
