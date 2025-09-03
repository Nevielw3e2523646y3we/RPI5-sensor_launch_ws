#!/usr/bin/env python3
import math, rclpy, argparse
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

INT32_MAX  =  0x7FFFFFFF
INT32_MIN  = -0x80000000
UINT32_MOD =  0x100000000

def q_from_yaw(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw/2.0), w=math.cos(yaw/2.0))

class EncoderOdom(Node):
    def __init__(self, args):
        super().__init__('encoder_odom')

        # args / params
        self.ticks_per_rev  = int(args.ticks_per_rev)
        self.wheel_diam     = float(args.wheel_diameter)
        self.track_width    = float(args.track_width)
        self.left_scale     = float(args.left_scale)
        self.right_scale    = float(args.right_scale)
        self.left_topic     = args.left_topic
        self.right_topic    = args.right_topic
        self.odom_topic     = args.odom_topic
        self.frame_id       = args.odom_frame
        self.child_frame_id = args.base_link_frame
        self.pub_hz         = float(args.publish_rate)

        self.m_per_tick = math.pi * self.wheel_diam / self.ticks_per_rev

        self.last_l_pub = None
        self.last_r_pub = None
        self.curr_l = None
        self.curr_r = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_pub_time = None

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Int32, self.left_topic,  self.cb_l, sensor_qos)
        self.create_subscription(Int32, self.right_topic, self.cb_r, sensor_qos)
        self.pub = self.create_publisher(Odometry, self.odom_topic, odom_qos)

        period = 1.0 / max(1.0, self.pub_hz)
        self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"encoder_odom: pub={self.pub_hz:.1f} Hz, "
            f"tpr={self.ticks_per_rev}, m_per_tick={self.m_per_tick:.9f} m, "
            f"track_width={self.track_width:.3f} m, "
            f"scales(L,R)=({self.left_scale:.6f},{self.right_scale:.6f})"
        )

    @staticmethod
    def wrap_int32_delta(curr: int, last: int) -> int:
        d = curr - last
        if d > INT32_MAX:
            d -= UINT32_MOD
        elif d < INT32_MIN:
            d += UINT32_MOD
        return int(d)

    def cb_l(self, msg: Int32):
        self.curr_l = int(msg.data)

    def cb_r(self, msg: Int32):
        self.curr_r = int(msg.data)

    def on_timer(self):
        now = self.get_clock().now()
        if self.curr_l is None or self.curr_r is None:
            self.last_pub_time = now
            self.last_l_pub = self.curr_l
            self.last_r_pub = self.curr_r
            return

        if self.last_pub_time is None or self.last_l_pub is None or self.last_r_pub is None:
            self.last_pub_time = now
            self.last_l_pub = self.curr_l
            self.last_r_pub = self.curr_r
            return

        dt = (now - self.last_pub_time).nanoseconds * 1e-9
        if dt <= 0:
            return
        dt = max(dt, 1e-3)

        dl_ticks = self.wrap_int32_delta(self.curr_l, self.last_l_pub)
        dr_ticks = self.wrap_int32_delta(self.curr_r, self.last_r_pub)

        self.last_l_pub = self.curr_l
        self.last_r_pub = self.curr_r
        self.last_pub_time = now

        s_l  = dl_ticks * self.m_per_tick * self.left_scale
        s_r  = dr_ticks * self.m_per_tick * self.right_scale
        s    = 0.5 * (s_r + s_l)
        dyaw = (s_r - s_l) / self.track_width

        yaw_mid = self.yaw + 0.5 * dyaw
        self.x  += s * math.cos(yaw_mid)
        self.y  += s * math.sin(yaw_mid)
        self.yaw += dyaw

        # wrap yaw to [-pi, pi)
        self.yaw = (self.yaw + math.pi) % (2*math.pi) - math.pi

        vx   = s / dt
        vyaw = dyaw / dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id  = self.child_frame_id
        odom.pose.pose.orientation = q_from_yaw(self.yaw)
        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.position.z  = 0.0

        # --- CLEAN, DIAGONAL COVARIANCES (no off-diagonal terms) ---
        # Pose: [x, y, z, roll, pitch, yaw]
        x_var = (0.05)**2
        y_var = (0.05)**2
        z_var = 1e6
        roll_var = 1e6
        pitch_var = 1e6
        yaw_var = (0.1)**2
        odom.pose.covariance = [
            x_var, 0,     0,      0,       0,        0,
            0,     y_var, 0,      0,       0,        0,
            0,     0,     z_var,  0,       0,        0,
            0,     0,     0,      roll_var,0,        0,
            0,     0,     0,      0,       pitch_var,0,
            0,     0,     0,      0,       0,        yaw_var
        ]

        # Twist: [vx, vy, vz, vroll, vpitch, vyaw]
        vx_var = (0.1)**2
        vy_var = 1e6
        vz_var = 1e6
        vroll_var = 1e6
        vpitch_var = 1e6
        vyaw_var = (0.2)**2
        odom.twist.covariance = [
            vx_var, 0,      0,        0,         0,          0,
            0,      vy_var, 0,        0,         0,          0,
            0,      0,      vz_var,   0,         0,          0,
            0,      0,      0,        vroll_var, 0,          0,
            0,      0,      0,        0,         vpitch_var, 0,
            0,      0,      0,        0,         0,          vyaw_var
        ]

        odom.twist.twist.linear.x  = float(vx)
        odom.twist.twist.angular.z = float(vyaw)

        self.pub.publish(odom)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ticks_per_rev',   type=int,   required=True)
    ap.add_argument('--wheel_diameter',  type=float, required=True)
    ap.add_argument('--track_width',     type=float, required=True)
    ap.add_argument('--left_scale',      type=float, required=True)
    ap.add_argument('--right_scale',     type=float, required=True)
    ap.add_argument('--left_topic',      default='/encoders/left')
    ap.add_argument('--right_topic',     default='/encoders/right')
    ap.add_argument('--odom_topic',      default='/wheel/odometry')
    ap.add_argument('--odom_frame',      default='odom')
    ap.add_argument('--base_link_frame', default='base_link')
    ap.add_argument('--publish_rate',    type=float, default=20.0)  # Hz
    args = ap.parse_args()

    rclpy.init()
    node = EncoderOdom(args)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
