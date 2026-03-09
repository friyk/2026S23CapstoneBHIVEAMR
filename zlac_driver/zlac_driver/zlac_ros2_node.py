import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

# Import your provided controller class
from .ZLAC8015D import Controller

class ZLACDriverNode(Node):
    def __init__(self):
        super().__init__('zlac_driver_node')
        
        # --- Parameters ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('wheel_base', 0.26) # The distance between your wheels in meters
        self.declare_parameter('wheel_radius', 0.105) 
        
        port = self.get_parameter('port').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # --- Initialize Controller ---
        self.get_logger().info(f"Connecting to motor driver on {port}...")
        self.controller = Controller(port=port)
        self.controller.disable_motor()
        self.controller.set_mode(3) # 3 = Velocity Control Mode
        self.controller.enable_motor()
        self.get_logger().info("Motors enabled in Velocity Mode.")
        
        # Odomsetup
        # Robot physical state variables (x, y, theta)
        self.x = 0
        self.y = 0
        self.th = 0
        
        # Timing State
        self.last_time = self.get_clock().now()
        
        # Read the initial wheel positions so we start calculating from 0
        self.last_l_travel, raw_r_travel = self.controller.get_wheels_travelled()
        # Mirror the right wheel just like velocity
        self.last_r_travel = -raw_r_travel
        
        # Timer to calculate and publish odom at 10Hz (0.1s)
        self.odom_timer = self.create_timer(0.1, self.publish_odom)

        # --- ROS 2 Subscriber ---
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # --- Current Publisher ---
        self.current_pub = self.create_publisher(
            Float32MultiArray,
            '/motor_amps',
            10
        )
        
        # --- Odom Publisher & TF Broadcaster ---
        self.odom_pub = self.create_publisher(
        	Odometry,
        	'/odom',
        	10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to read and publish current ever 0.5s (2Hz)
        self.current_timer = self.create_timer(0.5, self.publish_current)
    
    
    # odom publish function to calc differential drive
    def publish_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt == 0:
            return

        try:
            # 1. Read the absolute travel distances from the Modbus driver
            current_l_travel, raw_r_travel = self.controller.get_wheels_travelled()
            current_r_travel = -raw_r_travel # Invert right wheel
        except Exception as e:
            self.get_logger().error(f"Odom read failed: {e}")
            return

        # 2. Calculate how far each wheel has moved since the last loop
        d_l = current_l_travel - self.last_l_travel
        d_r = current_r_travel - self.last_r_travel

        self.last_l_travel = current_l_travel
        self.last_r_travel = current_r_travel

        # 3. Differential drive kinematics
        ds = (d_l + d_r) / 2.0
        dth = (d_r - d_l) / self.wheel_base

        # Update the robot's global position
        self.x += ds * math.cos(self.th + dth / 2.0)
        self.y += ds * math.sin(self.th + dth / 2.0)
        self.th += dth

        # Calculate instantaneous velocities
        v_x = ds / dt
        omega_z = dth / dt
        
        self.last_time = current_time

        # Convert the Theta (yaw) angle to a Quaternion for ROS 2 
        q_x = 0.0
        q_y = 0.0
        q_z = math.sin(self.th / 2.0)
        q_w = math.cos(self.th / 2.0)

        # 4. Publish the Coordinate Transform (TF)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q_x
        t.transform.rotation.y = q_y
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        self.tf_broadcaster.sendTransform(t)

        # 5. Publish the Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q_x
        odom.pose.pose.orientation.y = q_y
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w

        odom.twist.twist.linear.x = v_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega_z

        self.odom_pub.publish(odom)




    def cmd_vel_callback(self, msg):
        v_x = msg.linear.x
        omega_z = msg.angular.z
        
                
        # Differential drive kinematics
        v_L = v_x - (omega_z * self.wheel_base / 2.0)
        v_R = v_x + (omega_z * self.wheel_base / 2.0)
        
        # Convert linear velocity (m/s) to RPM
        rpm_L = (v_L / (2.0 * math.pi * self.wheel_radius)) * 60.0
        rpm_R = (v_R / (2.0 * math.pi * self.wheel_radius)) * 60.0
        
        # Note: Your script implies the right motor is physically mirrored 
        # (negated in get_linear_velocities), so we invert right RPM to drive forward.
        self.controller.set_rpm(int(rpm_L), int(-rpm_R))
       
    def publish_current(self):
        try:
            amp_L, amp_R = self.controller.get_current()
            
            # Package the two floats into a standard array message
            msg = Float32MultiArray()
            msg.data = [float(amp_L), float(amp_R)]
            self.current_pub.publish(msg)
            
            # Optional: Log a warning directly to the terminal if amps spike
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
        # Safety fallback: stop motors on shutdown
        node.controller.set_rpm(0, 0)
        node.controller.disable_motor()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
