"""
pedal_node.py
PCSensor FootSwitch (3-pedal) → ROS2 sensor_msgs/Joy

Device: PCsensor FootSwitch Keyboard (USB HID, appears as keyboard)
  Left   pedal → KEY_A (code 30) → buttons[0]  (teleop engage)
  Middle pedal → KEY_B (code 48) → buttons[1]  (recording toggle)
  Right  pedal → KEY_C (code 46) → buttons[2]  (spare)

Published topic:
  /teleop/pedal  sensor_msgs/Joy

Permissions:
  sudo usermod -aG input $USER  (then re-login)
  or run with: sudo ros2 run pedal_ros2 pedal_node
"""

import threading
import evdev
from evdev import InputDevice, ecodes

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

DEVICE_NAME = 'PCSensor FootSwitch Keyboard'

KEY_MAP = {
    ecodes.KEY_A: 0,  # Left  pedal → buttons[0]
    ecodes.KEY_B: 1,  # Middle pedal → buttons[1]
    ecodes.KEY_C: 2,  # Right  pedal → buttons[2]
}


def find_device(name: str) -> InputDevice:
    for path in evdev.list_devices():
        dev = InputDevice(path)
        if dev.name == name:
            return dev
    raise RuntimeError(f"Device '{name}' not found. Is it plugged in?")


class PedalNode(Node):

    def __init__(self):
        super().__init__('pedal_node')

        self.declare_parameter('device_name', DEVICE_NAME)
        self.declare_parameter('topic',       '/teleop/pedal')

        dev_name  = self.get_parameter('device_name').get_parameter_value().string_value
        topic     = self.get_parameter('topic').get_parameter_value().string_value

        self._pub   = self.create_publisher(Joy, topic, 10)
        self._state = [0, 0, 0]   # buttons[0..2]

        try:
            self._dev = find_device(dev_name)
            self.get_logger().info(f"Found device: {self._dev.path} — {self._dev.name}")
        except RuntimeError as e:
            self.get_logger().error(str(e))
            raise

        # Read evdev in background thread (blocking read_loop)
        t = threading.Thread(target=self._read_loop, daemon=True)
        t.start()

        self.get_logger().info(f"Publishing to {topic}")

    def _read_loop(self):
        for event in self._dev.read_loop():
            if event.type != ecodes.EV_KEY:
                continue
            if event.code not in KEY_MAP:
                continue

            btn_idx = KEY_MAP[event.code]
            # value: 1=press, 0=release, 2=repeat(hold) → treat repeat as press
            self._state[btn_idx] = 1 if event.value >= 1 else 0
            self._publish()

    def _publish(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.buttons = list(self._state)
        msg.axes = []
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PedalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
