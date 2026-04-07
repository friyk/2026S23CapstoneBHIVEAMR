import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

OBSTACLE_DISTANCE = 0.5  # metres ? stop if anything closer than this
FORWARD_SPEED = 0.2      # m/s ? forward speed when following
ANGULAR_SPEED = 0.0      # no turning, straight follow only

class FollowNode(Node):
    def __init__(self):
        super().__init__('follow_node')

        # Publisher ? velocity commands to motors
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber ? BLE commands from phone
        self.ble_sub = self.create_subscription(
            String, '/ble/commands', self.ble_callback, 10
        )

        # Subscriber ? LiDAR scan for obstacle detection
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.is_following = False
        self.obstacle_detected = False

        # Timer ? publishes velocity at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Follow node started')
        self.get_logger().info(f'Obstacle stop distance: {OBSTACLE_DISTANCE}m')
        self.get_logger().info(f'Forward speed: {FORWARD_SPEED} m/s')

    def ble_callback(self, msg):
        command = msg.data.strip()
        self.get_logger().info(f'[BLE] Received command: {command}')

        if command == 'start_follow':
            self.is_following = True
            self.get_logger().info('[FOLLOW] Started following')

        elif command in ('stop_follow', 'emergency_stop'):
            self.is_following = False
            self.stop_robot()
            self.get_logger().info(f'[FOLLOW] Stopped ? {command}')

    def scan_callback(self, msg):
        # Check front-facing LiDAR readings (centre 60 degrees)
        # LaserScan ranges go from angle_min to angle_max
        # We check indices corresponding to roughly -30 to +30 degrees
        total = len(msg.ranges)
        front_start = int(total * 0.0)   # 0 degrees
        front_end   = int(total * 0.083) # ~30 degrees
        back_start  = int(total * 0.917) # ~330 degrees

        front_ranges = (
            msg.ranges[front_start:front_end] +
            msg.ranges[back_start:]
        )

        # Filter out invalid readings (0.0 or inf)
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
        twist = Twist()

        if self.is_following and not self.obstacle_detected:
            twist.linear.x = FORWARD_SPEED
            twist.angular.z = ANGULAR_SPEED
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
