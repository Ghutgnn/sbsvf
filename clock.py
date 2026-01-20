#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time


class ClockPublisher(Node):
    def __init__(self):
        super().__init__("clock_publisher")

        self.declare_parameter("dt", 0.02)  # seconds per tick
        dt = float(self.get_parameter("dt").value)
        if dt <= 0.0:
            raise ValueError("dt must be > 0")

        self.dt_ns = int(round(dt * 1e9))
        self.t_ns = 0  # start at 0

        self.pub = self.create_publisher(Clock, "/clock", 10)
        self.timer = self.create_timer(dt, self.on_timer)

        self.get_logger().info(f"/clock publishing, dt={dt}s ({self.dt_ns}ns)")

    def on_timer(self):
        msg = Clock()
        sec = self.t_ns // 1_000_000_000
        nsec = self.t_ns % 1_000_000_000
        msg.clock = Time(sec=int(sec), nanosec=int(nsec))
        self.pub.publish(msg)
        self.t_ns += self.dt_ns


def main():
    rclpy.init()
    node = ClockPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
