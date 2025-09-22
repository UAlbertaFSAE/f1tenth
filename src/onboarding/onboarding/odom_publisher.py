from typing import Optional as opt

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import (
    FloatingPointRange,
    ParameterDescriptor,
    SetParametersResult,
)
from rclpy.node import Node


class OdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("odom_publisher")

        # 
        v_desc = ParameterDescriptor(
            description="linear speed",
            floating_point_range=[
                FloatingPointRange(from_value=-10.0, to_value=10.0, step=0.01)
            ],
        )
        d_desc = ParameterDescriptor(
            description="steering angle",
            floating_point_range=[
                FloatingPointRange(from_value=-3.14, to_value=3.14, step=0.001)
            ],
        )

        # declare parameters with ranges and defaults
        self.declare_parameter("v", 0.0, v_desc)
        self.declare_parameter("d", 0.0, d_desc)

        # internal copies of the parameter values
        self.v = float(self.get_parameter("v").value)
        self.d = float(self.get_parameter("d").value)

        # publisher
        self.publisher_ = self.create_publisher(AckermannDriveStamped, "drive", 10)

        # 1 kHz timer
        self.timer = self.create_timer(0.001, self.timer_callback)

        # update internal values when parameters change
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # avoid flooding logs at 1kHz; log a sample occasionally
        self._publish_count = 0
        self.get_logger().info(
            f"Initialized odom_publisher with v={self.v}, d={self.d}"
        )

    def _on_set_parameters(self, params: list) -> SetParametersResult:
        for p in params:
            if p.name == "v":
                self.v = float(p.value)
            elif p.name == "d":
                self.d = float(p.value)
        return SetParametersResult(successful=True)

    def timer_callback(self) -> None:
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = self.v
        msg.drive.steering_angle = self.d
        self.publisher_.publish(msg)

        self._publish_count += 1
        if self._publish_count % 1000 == 0:
            self.get_logger().info(f"Publishing sample v={self.v}, d={self.d}")


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
