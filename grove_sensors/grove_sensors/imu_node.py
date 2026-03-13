#!/usr/bin/env python3
import rclpy, smbus2, math, time
from rclpy.node import Node
from sensor_msgs.msg import Imu

# ISM330DHCX registers
ADDR         = 0x6a
WHO_AM_I     = 0x0F
CTRL1_XL     = 0x10  # Accel control
CTRL2_G      = 0x11  # Gyro control
OUTX_L_G     = 0x22  # Gyro X low byte
OUTX_L_A     = 0x28  # Accel X low byte

def read_word(bus, addr, reg):
    low  = bus.read_byte_data(addr, reg)
    high = bus.read_byte_data(addr, reg + 1)
    val  = (high << 8) | low
    return val - 65536 if val >= 32768 else val

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_addr', ADDR)
        bus_num   = self.get_parameter('i2c_bus').value
        self.addr = self.get_parameter('i2c_addr').value

        self.bus = smbus2.SMBus(bus_num)

        # Enable accel: 104Hz, 2g range (0x40)
        self.bus.write_byte_data(self.addr, CTRL1_XL, 0x40)
        # Enable gyro: 104Hz, 250dps range (0x40)
        self.bus.write_byte_data(self.addr, CTRL2_G, 0x40)
        time.sleep(0.1)

        # Calibrate gyro bias — keep robot still for 2 seconds
        self.get_logger().info('Calibrating IMU gyro bias — keep robot still...')
        samples = []
        for _ in range(200):
            gz = read_word(self.bus, self.addr, OUTX_L_G + 4)
            samples.append(gz)
            time.sleep(0.01)
        self.gyro_z_bias = sum(samples) / len(samples)
        self.get_logger().info(f'Gyro Z bias: {self.gyro_z_bias:.2f} raw counts')

        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.create_timer(0.01, self.publish_imu)  # 100Hz
        self.get_logger().info('IMU node ready on bus 7, addr 0x6a')

    def publish_imu(self):
        # ISM330DHCX: accel sensitivity 2g = 0.061 mg/LSB
        # gyro sensitivity 250dps = 8.75 mdps/LSB
        ax = read_word(self.bus, self.addr, OUTX_L_A)     * 0.061e-3 * 9.81
        ay = read_word(self.bus, self.addr, OUTX_L_A + 2) * 0.061e-3 * 9.81
        az = read_word(self.bus, self.addr, OUTX_L_A + 4) * 0.061e-3 * 9.81
        gx = read_word(self.bus, self.addr, OUTX_L_G)     * 8.75e-3 * math.pi/180
        gy = read_word(self.bus, self.addr, OUTX_L_G + 2) * 8.75e-3 * math.pi/180
        gz = (read_word(self.bus, self.addr, OUTX_L_G + 4) - self.gyro_z_bias) * 8.75e-3 * math.pi/180

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.orientation_covariance[0]          = -1.0  # orientation unknown
        msg.linear_acceleration_covariance[0]  = 0.001
        msg.linear_acceleration_covariance[4]  = 0.001
        msg.linear_acceleration_covariance[8]  = 0.001
        msg.angular_velocity_covariance[0]     = 0.0001
        msg.angular_velocity_covariance[4]     = 0.0001
        msg.angular_velocity_covariance[8]     = 0.0001
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ImuNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
