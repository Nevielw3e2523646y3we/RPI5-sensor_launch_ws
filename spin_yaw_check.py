#!/usr/bin/env python3
import argparse, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

def make_qos(reliability: str, depth: int = 100):
    q = QoSProfile(depth=depth)
    q.history = HistoryPolicy.KEEP_LAST
    q.durability = DurabilityPolicy.VOLATILE
    q.reliability = (ReliabilityPolicy.BEST_EFFORT
                     if reliability.lower() == 'best_effort'
                     else ReliabilityPolicy.RELIABLE)
    return q

def trapz(ts, xs):
    ts = np.asarray(ts, float); xs = np.asarray(xs, float)
    return float(np.trapz(xs, ts)) if ts.size >= 2 else 0.0

# ---------- SPIN MODE ----------
class SpinCheck(Node):
    def __init__(self, imu_topic, odom_topic, duration_s, bias_z, imu_rel, odom_rel, track_width):
        super().__init__('spin_yaw_check')
        self.bias = bias_z
        self.duration_s = duration_s
        self.track_width = track_width
        self.t0 = None
        self.t_imu, self.gz = [], []
        self.t_wo,  self.vyaw = [], []
        self.create_subscription(Imu, imu_topic, self.on_imu, make_qos(imu_rel))
        self.create_subscription(Odometry, odom_topic, self.on_odom, make_qos(odom_rel))

    def on_imu(self, m: Imu):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None: self.t0 = t
        tt = t - self.t0
        self.t_imu.append(tt)
        self.gz.append(float(m.angular_velocity.z) - self.bias)
        if tt >= self.duration_s:
            self.finish()

    def on_odom(self, m: Odometry):
        if self.t0 is None: return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        tt = t - self.t0
        self.t_wo.append(tt)
        self.vyaw.append(float(m.twist.twist.angular.z))

    def finish(self):
        yaw_imu_rad  = trapz(self.t_imu, self.gz)
        yaw_odom_rad = trapz(self.t_wo,  self.vyaw)
        yaw_imu_deg  = yaw_imu_rad  * 180.0 / math.pi
        yaw_odom_deg = yaw_odom_rad * 180.0 / math.pi
        k_ang = (yaw_imu_rad / yaw_odom_rad) if abs(yaw_odom_rad) > 1e-6 else float('nan')

        print("\n=== Spin yaw-scale check ===")
        print(f"Integrated yaw IMU : {yaw_imu_deg:+.2f} deg")
        print(f"Integrated yaw ODOM: {yaw_odom_deg:+.2f} deg")
        print(f"Angular gain k_ang = yaw_IMU / yaw_ODOM = {k_ang:.6f}")
        if self.track_width and math.isfinite(k_ang):
            L_new = float(self.track_width) / k_ang
            print(f"\nIf odom uses track width L = {float(self.track_width):.4f} m:")
            print(f"  -> set L_new = L_old / k_ang = {L_new:.4f} m")
        print("\nApply k_ang by either:")
        print("  • Adjusting effective track width in your odom node, or")
        print("  • Multiplying computed vyaw by k_ang before publishing.")
        rclpy.shutdown()

# ---------- LINEAR MODE ----------
class LinearCheck(Node):
    def __init__(self, odom_topic, duration_s, odom_rel, true_dist):
        super().__init__('linear_check')
        self.duration_s = duration_s
        self.true_dist = true_dist
        self.t0 = None
        self.t, self.vx = [], []
        self.create_subscription(Odometry, odom_topic, self.on_odom, make_qos(odom_rel))

    def on_odom(self, m: Odometry):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None: self.t0 = t
        tt = t - self.t0
        self.t.append(tt)
        self.vx.append(float(m.twist.twist.linear.x))
        if tt >= self.duration_s:
            self.finish()

    def finish(self):
        D_odom = trapz(self.t, self.vx)
        print("\n=== Linear distance check ===")
        print(f"Integrated odom distance: {D_odom:.3f} m over ~{self.t[-1]:.1f}s")
        if self.true_dist is not None and self.true_dist > 0:
            k_lin = float(self.true_dist) / D_odom if abs(D_odom) > 1e-6 else float('nan')
            print(f"Ground truth distance:   {float(self.true_dist):.3f} m")
            print(f"Linear gain k_linear = D_true / D_odom = {k_lin:.6f}")
            print("\nApply k_linear by either:")
            print("  • Adjusting wheel radius/counts-per-meter in odom node, or")
            print("  • Multiplying published vx by k_linear.")
        print("\nTip: do 3–5 runs and average k_linear.")
        rclpy.shutdown()

def main():
    ap = argparse.ArgumentParser(description="IMU/odom calibration (spin + linear) with QoS controls")
    sub = ap.add_subparsers(dest='cmd', required=True)

    sp = sub.add_parser('spin', help='Compare IMU vs wheel yaw during a 360° spin')
    sp.add_argument('--imu',  default='/imu/data_ahrs')
    sp.add_argument('--odom', default='/wheel/odometry')
    sp.add_argument('--duration', type=float, default=25.0)
    sp.add_argument('--bias', type=float, default=0.0, help='bias to subtract from IMU z (rad/s)')
    sp.add_argument('--track-width', type=float, default=None, help='current odom track width (m)')
    sp.add_argument('--imu-reliability',  choices=['best_effort','reliable'], default='reliable')
    sp.add_argument('--odom-reliability', choices=['best_effort','reliable'], default='best_effort')

    ln = sub.add_parser('linear', help='Integrate odom vx on a straight run')
    ln.add_argument('--odom', default='/wheel/odometry')
    ln.add_argument('--duration', type=float, default=6.0)
    ln.add_argument('--odom-reliability', choices=['best_effort','reliable'], default='best_effort')
    ln.add_argument('--distance', type=float, default=None, help='Tape-measured distance in metres')

    args = ap.parse_args()
    rclpy.init()

    if args.cmd == 'spin':
        node = SpinCheck(args.imu, args.odom, args.duration, args.bias,
                         args.imu_reliability, args.odom_reliability, args.track_width)
    else:
        node = LinearCheck(args.odom, args.duration, args.odom_reliability, args.distance)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
