#!/usr/bin/env python3
"""Reads CSV ultrasonic frames from ESP32 over USB serial and
publishes one sensor_msgs/Range per sensor."""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Range
import serial

SENSOR_ORDER = ['fl', 'fr', 'br', 'bl']

# Per-sensor config: frame_id, max_range (m), field_of_view (rad).
# Front sensors get a longer look-ahead; rear only watches for close threats.
SENSOR_CFG = {
    'fl': {'frame': 'us_fl_link', 'max_range': 1.50, 'fov': math.radians(15)},
    'fr': {'frame': 'us_fr_link', 'max_range': 1.50, 'fov': math.radians(15)},
    'br': {'frame': 'us_br_link', 'max_range': 0.60, 'fov': math.radians(15)},
    'bl': {'frame': 'us_bl_link', 'max_range': 0.60, 'fov': math.radians(15)},
}
MIN_RANGE = 0.03  # HC-SR04 dead zone


class UltrasonicBridge(Node):
    def __init__(self):
        super().__init__('ultrasonic_bridge')
        self.declare_parameter('port', '/dev/esp32')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pubs = {}
        for key in SENSOR_ORDER:
            self.pubs[key] = self.create_publisher(
                Range, f'/ultrasonic/{key}', qos)

        self.get_logger().info(f'Opening {port} @ {baud}')
        self.ser = serial.Serial(port, baud, timeout=1.0)
        self.ser.reset_input_buffer()

        self.timer = self.create_timer(0.0, self.spin_serial)

    def spin_serial(self):
        try:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
        except Exception as e:
            self.get_logger().warn(f'serial read error: {e}')
            return
        if not line or not line.startswith('US,'):
            return
        parts = line.split(',')
        if len(parts) != 5:
            return
        try:
            vals_cm = [float(x) for x in parts[1:]]
        except ValueError:
            return

        stamp = self.get_clock().now().to_msg()
        for key, cm in zip(SENSOR_ORDER, vals_cm):
            cfg = SENSOR_CFG[key]
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = cfg['frame']
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = cfg['fov']
            msg.min_range = MIN_RANGE
            msg.max_range = cfg['max_range']
            if cm < 0:
                msg.range = cfg['max_range']   # timeout -> treat as clear
            else:
                m = cm / 100.0
                msg.range = max(MIN_RANGE, min(m, cfg['max_range']))
            self.pubs[key].publish(msg)


def main():
    rclpy.init()
    node = UltrasonicBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
