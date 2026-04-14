import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import os

OBSTACLE_DISTANCE = 0.5   # metres ? LiDAR stop threshold
TARGET_DISTANCE   = 1.5   # metres ? ideal following distance
STOP_DISTANCE     = 1.0   # metres ? stop if phone closer than this
FAST_SPEED        = 0.3   # m/s ? phone is far
SLOW_SPEED        = 0.15  # m/s ? phone is close but not too close
COMMAND_FILE      = '/tmp/ble_command.txt'

class FollowNode(Node):
    def __init__(self):
        super().__init__('follow_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.is_following    = False
        self.obstacle_detected = False
        self.phone_distance  = 999.0  # default far away

        self.timer = self.create_timer(0.1, self.control_loop)

        # Poll command file every 0.2s for BLE commands and distance
        self.create_timer(0.2, self.read_command_file)

        self.get_logger().info('Follow node started')
        self.get_logger().info(f'Obstacle stop distance: {OBSTACLE_DISTANCE}m')
        self.get_logger().info(f'Target following distance: {TARGET_DISTANCE}m')
        self.get_logger().info(f'Stop distance: {STOP_DISTANCE}m')

    def read_command_file(self):
        """Read BLE commands and distance updates from file bridge"""
        try:
            if not os.path.exists(COMMAND_FILE):
                return
            with open(COMMAND_FILE, 'r') as f:
                msg = f.read().strip()
            if not msg:
                return

            # Clear file after reading
            with open(COMMAND_FILE, 'w') as f:
                f.write('')

            self.process_message(msg)

        except Exception as e:
            self.get_logger().error(f'[FILE] Error reading command file: {e}')

    def process_message(self, msg):
        """Process BLE command or distance update"""

        # Distance update from phone
        if msg.startswith('distance:'):
            try:
                distance = float(msg.split(':')[1])
                self.phone_distance = distance
                self.get_logger().info(f'[DISTANCE] Phone distance: {distance:.2f}m')
            except ValueError:
                self.get_logger().warn(f'[DISTANCE] Invalid distance: {msg}')

        # Control commands
        elif msg == 'start_follow':
            self.is_following = True
            self.phone_distance = 999.0  # reset distance
            self.get_logger().info('[FOLLOW] Started following')

        elif msg in ('stop_follow', 'emergency_stop'):
            self.is_following = False
            self.stop_robot()
            self.get_logger().info(f'[FOLLOW] Stopped ? {msg}')

    def scan_callback(self, msg):
        """LiDAR obstacle detection ? front 60 degrees"""
        total = len(msg.ranges)
        front_start = int(total * 0.0)
        front_end   = int(total * 0.083)
        back_start  = int(total * 0.917)

        front_ranges = (
            msg.ranges[front_start:front_end] +
            msg.ranges[back_start:]
        )

        valid = [r for r in front_ranges if 0.01 < r < msg.range_max]

        if valid and min(valid) < OBSTACLE_DISTANCE:
            if not self.obstacle_detected:
                self.get_logger().warn(
                    f'[OBSTACLE] Detected at {min(valid):.2f}m ? stopping'
                )
            self.obstacle_detected = True
        else:
            if self.obstacle_detected:
                self.get_logger().info('[OBSTACLE] Cleared ? resuming')
            self.obstacle_detected = False

    def control_loop(self):
        """Main control loop ? runs at 10Hz"""
        twist = Twist()

        if self.is_following and not self.obstacle_detected:
            if self.phone_distance <= STOP_DISTANCE:
                # Phone too close ? stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif self.phone_distance <= TARGET_DISTANCE:
                # Phone within target range ? move slow
                twist.linear.x = SLOW_SPEED
                twist.angular.z = 0.0
            else:
                # Phone far away ? move fast
                twist.linear.x = FAST_SPEED
                twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
