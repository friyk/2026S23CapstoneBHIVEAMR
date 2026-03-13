#!/usr/bin/env python3
import rclpy, time, gpiod
from rclpy.node import Node
from sensor_msgs.msg import Range

# Verified gpiochip0 line numbers for Jetson Orin Nano + Grove Base Hat for RPi
# Grove label : (echo_line, trig_line)
SENSORS = {
    # Changinginginging to esp32 so hold the fking phone
    # 'front': (124, 122),   # D26=echo, D27=trig
    # 'back':  (50,  53),    # D18=echo, D19=trig
    # Uncomment when remaining sensors arrive:
    # 'left':  (43,  85),  # D24=echo, D24+1=trig
    # 'right': (113, 106), # D16=echo, D16+1=trig
}

CHIP = '/dev/gpiochip0'

def measure_distance(chip_path, echo_line, trig_line):
    """Single HC-SR04 measurement using gpiod 2.x API"""
    try:
        # Trigger pulse
        with gpiod.request_lines(
            chip_path, consumer='trig',
            config={trig_line: gpiod.LineSettings(
                direction=gpiod.line.Direction.OUTPUT,
                output_value=gpiod.line.Value.INACTIVE
            )}
        ) as trig:
            trig.set_value(trig_line, gpiod.line.Value.INACTIVE)
            time.sleep(0.000002)
            trig.set_value(trig_line, gpiod.line.Value.ACTIVE)
            time.sleep(0.00001)   # 10us pulse
            trig.set_value(trig_line, gpiod.line.Value.INACTIVE)

        # Measure echo pulse width
        with gpiod.request_lines(
            chip_path, consumer='echo',
            config={echo_line: gpiod.LineSettings(
                direction=gpiod.line.Direction.INPUT
            )}
        ) as echo:
            timeout = time.time() + 0.03
            while echo.get_value(echo_line) == gpiod.line.Value.INACTIVE:
                if time.time() > timeout:
                    return float('inf')
            start = time.time()

            timeout = time.time() + 0.03
            while echo.get_value(echo_line) == gpiod.line.Value.ACTIVE:
                if time.time() > timeout:
                    return float('inf')
            end = time.time()

        distance = (end - start) * 34300 / 2  # cm
        return distance / 100.0  # metres

    except Exception as e:
        return float('inf')

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.pubs = {}
        for name in SENSORS:
            self.pubs[name] = self.create_publisher(
                Range, f'ultrasonic/{name}', 10)
        self.create_timer(0.05, self.publish_ranges)  # 20Hz
        self.get_logger().info(f'Ultrasonic node started for sensors: {list(SENSORS.keys())}')

    def publish_ranges(self):
        for name, (echo, trig) in SENSORS.items():
            dist = measure_distance(CHIP, echo, trig)
            msg = Range()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = f'ultrasonic_{name}_link'
            msg.radiation_type  = Range.ULTRASOUND
            msg.field_of_view   = 0.26   # ~15 degrees
            msg.min_range       = 0.02
            msg.max_range       = 4.00
            msg.range = max(msg.min_range, min(dist, msg.max_range))
            self.pubs[name].publish(msg)

def main():
    rclpy.init()
    rclpy.spin(UltrasonicNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
