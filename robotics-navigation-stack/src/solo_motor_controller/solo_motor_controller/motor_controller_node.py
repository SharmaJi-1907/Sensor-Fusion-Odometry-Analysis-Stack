#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

try:
    import SoloPy as solo
except Exception as import_error:  # pragma: no cover
    solo = None
    _import_error = import_error


class SoloMotorNode(Node):
    """ROS2 node that commands a SOLO motor controller and publishes encoder counts."""

    def __init__(
        self,
        node_name: str,
        default_encoder_topic: str,
        default_speed_cmd_topic: str,
        default_port: str = "/dev/ttyUSB0",
        default_addr: int = 0x01,
        default_loop_rate_hz: float = 50.0,
    ) -> None:
        super().__init__(node_name)

        if solo is None:
            self.get_logger().error(
                f"Failed to import SoloPy: {_import_error}. Install the SoloPy package to use this node."
            )
            raise _import_error

        # Parameters
        port: str = self.declare_parameter("motor_port", default_port).value
        addr: int = self.declare_parameter("motor_addr", default_addr).value
        self.loop_rate_hz: float = self.declare_parameter("loop_rate", default_loop_rate_hz).value
        self.encoder_topic: str = self.declare_parameter("encoder_topic", default_encoder_topic).value
        self.speed_cmd_topic: str = self.declare_parameter("speed_cmd_topic", default_speed_cmd_topic).value

        # Motor controller (SoloPy v4.1.0 UART API)
        self._solo = solo.SoloMotorControllerUart(port, addr)
        # Basic configuration to operate in speed mode with encoders
        self._solo.set_command_mode(solo.CommandMode.DIGITAL)
        self._solo.set_feedback_control_mode(solo.FeedbackControlMode.ENCODERS)
        self._solo.set_control_mode(solo.ControlMode.SPEED_MODE)
        self._current_speed: int = 0
        self._latest_cmd: int = 0

        # ROS interfaces
        self._pub_encoder = self.create_publisher(Int64, self.encoder_topic, 10)
        self._sub_speed = self.create_subscription(Int64, self.speed_cmd_topic, self._speed_callback, 10)

        # Timer loop
        self._timer = self.create_timer(1.0 / self.loop_rate_hz, self._control_loop)
        self.get_logger().info(
            f"Started {node_name} (port={port}, addr={hex(addr)}), "
            f"cmd_topic='{self.speed_cmd_topic}', enc_topic='{self.encoder_topic}'"
        )

    def _speed_callback(self, msg: Int64) -> None:
        self._latest_cmd = int(msg.data)

    def _control_loop(self) -> None:
        # Write speed if changed
        if self._latest_cmd != self._current_speed:
            self._set_speed(self._latest_cmd, self._current_speed)
            self._current_speed = self._latest_cmd

        # Try reading encoder, retry once if busy
        for _ in range(2):
            try:
                enc_val, _ = self._solo.get_position_counts_feedback()
                self._pub_encoder.publish(Int64(data=enc_val))
                break
            except Exception:
                time.sleep(0.002)  # 2 ms short wait

    def _set_speed(self, commanded_speed: int, previous_speed: int) -> None:
        if commanded_speed > 0:
            if previous_speed < 0:
                self._solo.set_motor_direction(solo.Direction.CLOCKWISE)
            self._solo.set_speed_reference(abs(commanded_speed))
        elif commanded_speed < 0:
            if previous_speed >= 0:
                self._solo.set_motor_direction(solo.Direction.COUNTERCLOCKWISE)
            self._solo.set_speed_reference(abs(commanded_speed))
        else:
            self._solo.set_speed_reference(0)
            if previous_speed < 0:
                self._solo.set_motor_direction(solo.Direction.CLOCKWISE)

    def destroy_node(self) -> None:  # type: ignore[override]
        try:
            self.get_logger().info(f"Shutting down {self.get_name()}...")
            self._solo.serial_close()
        finally:
            super().destroy_node()


def main_right(args=None) -> None:
    rclpy.init(args=args)
    node = SoloMotorNode(
        node_name="right_motor_node",
        default_encoder_topic="right_encoder_counts",
        default_speed_cmd_topic="right_motor_speed_cmd",
        default_addr=0x01,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_left(args=None) -> None:
    rclpy.init(args=args)
    node = SoloMotorNode(
        node_name="left_motor_node",
        default_encoder_topic="left_encoder_counts",
        default_speed_cmd_topic="left_motor_speed_cmd",
        default_addr=0x02,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    # Default to running the right motor node when invoked directly
    main_right()


