#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from riptide_msgs2.msg import FirmwareStatus


class FirmwareStatusPublisher(Node):
    def __init__(self):
        super().__init__("firmware_status_publisher")
        self.publisher = self.create_publisher(FirmwareStatus, "nros_fw_status_test_topic", 10)
        self.start_ms = 12345
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0, self.publish_status)  # 1 Hz

    def publish_status(self):
        elapsed_ms = (self.get_clock().now() - self.start_time).nanoseconds // 1_000_000

        msg = FirmwareStatus()
        msg.board_name = "fortnite"
        msg.bus_id = 33
        msg.client_id = 2
        msg.faults = 0xFFFFFFFF  # UINT32_MAX
        msg.kill_switches_asserting_kill = 14
        msg.kill_switches_enabled = 1
        msg.kill_switches_needs_update = 1303
        msg.kill_switches_timed_out = 14
        msg.uptime_ms = self.start_ms + elapsed_ms
        msg.version_major = 83
        msg.version_minor = 2
        msg.version_release_type = 9

        self.publisher.publish(msg)
        self.get_logger().info(f"Published status for '{msg.board_name}' (uptime {msg.uptime_ms} ms)")


def main(args=None):
    rclpy.init(args=args)
    node = FirmwareStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
