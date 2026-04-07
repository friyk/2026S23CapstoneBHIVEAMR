import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import math

from .ZLAC8015D import Controller

class ZLACDriverNode(Node):
    def __init__(self):
        super().__init__('zlac_driver_node')

        # --- Parameters ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('wheel_base', 0.275)
        self.declare_parameter('wheel_radius', 0.0865)

        port = self.get_parameter('port').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # --- Initialize Controller ---
        self.get_logger().info(f"Connecting to motor driver on {port}...")
        self.controller = Controller(port=port)
        self.controller.disable_motor()
        self.controller.set_mode(3)
        self.controller.enable_motor()
        self.get_logger().info("Motors enabled in Velocity Mode.")

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        self.last_l_travel, raw_r_travel = self.controller.get_wheels_travelled()
        self.last_r_travel = -raw_r_travel

        # --- Publishers ---
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.current_pub = self.create_publisher(Float32MultiArray, '/motor_amps', 10)

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # --- Timers ---
        self.odom_timer = self.create_timer(0.05, self.publish_odom)   # 20Hz
        self.current_timer = self.create_timer(0.5, self.publish_current)

    def publish_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt == 0:
            return

        try:
            current_l_travel, raw_r_travel = self.controller.get_wheels_travelled()
            current_r_travel = -raw_r_travel
        except Exception as e:
            self.get_logger().error(f"Odom read failed: {e}")
            return

        d_l = current_l_travel - self.last_l_travel
        d_r = current_r_travel - self.last_r_travel
        self.last_l_travel = current_l_travel
        self.last_r_travel = current_r_travel

        ds = (d_l + d_r) / 2.0
        dth = (d_r - d_l) / self.wheel_base

        self.x += ds * math.cos(self.th + dth / 2.0)
        self.y += ds * math.sin(self.th + dth / 2.0)
        self.th += dth

        v_x = ds / dt
        omega_z = dth / dt
        self.last_time = current_time

        q_z = math.sin(self.th / 2.0)
        q_w = math.cos(self.th / 2.0)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w

        # Pose covariance — yaw less trusted, EKF will correct with IMU
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[35] = 0.05   # yaw

        odom.twist.twist.linear.x = v_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega_z

        # Twist covariance
        odom.twist.covariance[0]  = 0.01  # vx
        odom.twist.covariance[35] = 0.05  # vyaw

        self.odom_pub.publish(odom)

    def cmd_vel_callback(self, msg):
        v_x = msg.linear.x
        omega_z = msg.angular.z

        v_L = v_x - (omega_z * self.wheel_base / 2.0)
        v_R = v_x + (omega_z * self.wheel_base / 2.0)

        rpm_L = (v_L / (2.0 * math.pi * self.wheel_radius)) * 60.0
        rpm_R = (v_R / (2.0 * math.pi * self.wheel_radius)) * 60.0

        self.controller.set_rpm(int(rpm_L), int(-rpm_R))

    def publish_current(self):
        try:
            amp_L, amp_R = self.controller.get_current()
            msg = Float32MultiArray()
            msg.data = [float(amp_L), float(amp_R)]
            self.current_pub.publish(msg)
            if abs(amp_L) > 15.0 or abs(amp_R) > 15.0:
                self.get_logger().warn(f"HIGH LOAD DETECTED: Left: {amp_L}A | Right: {amp_R}A")
        except Exception as e:
            self.get_logger().error(f"Failed to read current: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ZLACDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down motors...")
    finally:
        node.controller.set_rpm(0, 0)
        node.controller.disable_motor()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
