from __future__ import annotations

import json
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt8

try:
    import serial
    from serial import SerialException
except Exception:  # pragma: no cover - dependency availability is environment-specific
    serial = None
    SerialException = Exception


class UartDispenseBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_uart_dispense_bridge")

        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("flavor_selection_topic", "/flavor_selection")
        self.declare_parameter("currently_dispensing_topic", "/currently_dispensing")
        self.declare_parameter("poll_hz", 50.0)
        self.declare_parameter("write_timeout_sec", 0.2)
        self.declare_parameter("read_timeout_sec", 0.0)
        self.declare_parameter("initial_currently_dispensing", False)

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.flavor_selection_topic = str(self.get_parameter("flavor_selection_topic").value)
        self.currently_dispensing_topic = str(
            self.get_parameter("currently_dispensing_topic").value
        )
        poll_hz = max(1.0, float(self.get_parameter("poll_hz").value))
        self.write_timeout_sec = float(self.get_parameter("write_timeout_sec").value)
        self.read_timeout_sec = float(self.get_parameter("read_timeout_sec").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.currently_dispensing_pub = self.create_publisher(
            Bool,
            self.currently_dispensing_topic,
            qos,
        )
        self.create_subscription(
            UInt8,
            self.flavor_selection_topic,
            self._on_flavor_selection,
            qos,
        )

        self._serial: Optional[serial.Serial] = None if serial is not None else None
        self._read_buffer = bytearray()
        self._currently_dispensing = bool(
            self.get_parameter("initial_currently_dispensing").value
        )
        self._last_flavor_selection: Optional[int] = None

        self._connect_serial()
        self._publish_currently_dispensing(self._currently_dispensing, force=True)
        self.create_timer(1.0 / poll_hz, self._poll_serial)
        self.get_logger().info(
            f"UART dispense bridge listening on {self.port} at {self.baudrate} baud; "
            f"publishing dispensing state to {self.currently_dispensing_topic} and "
            f"reading flavor commands from {self.flavor_selection_topic}"
        )

    def _connect_serial(self) -> None:
        if serial is None:
            self.get_logger().error("pyserial is unavailable; UART dispense bridge is disabled")
            return

        try:
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.read_timeout_sec,
                write_timeout=self.write_timeout_sec,
            )
        except Exception as exc:
            self._serial = None
            self.get_logger().error(f"Failed to open UART port {self.port}: {exc}")

    def _publish_currently_dispensing(self, state: bool, *, force: bool = False) -> None:
        if not force and self._currently_dispensing == state:
            return

        self._currently_dispensing = state
        msg = Bool()
        msg.data = state
        self.currently_dispensing_pub.publish(msg)
        self.get_logger().info(f"currently_dispensing -> {'true' if state else 'false'}")

    def _on_flavor_selection(self, msg: UInt8) -> None:
        selection = int(msg.data)
        if selection not in (1, 2, 3):
            self.get_logger().warn(f"Ignoring unsupported flavor selection value {selection}")
            return

        payload = {"message_type": "flavor", "message": selection}
        if self._write_json(payload):
            if self._last_flavor_selection != selection:
                self.get_logger().info(f"Sent flavor selection {selection}")
                self._last_flavor_selection = selection

    def _write_json(self, payload: dict[str, object]) -> bool:
        if self._serial is None:
            self.get_logger().error(f"UART port {self.port} is not open; dropping {payload}")
            return False

        try:
            encoded = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
            self._serial.write(encoded)
            self._serial.flush()
            return True
        except SerialException as exc:
            self.get_logger().error(f"UART write failed: {exc}")
            self._serial = None
            return False
        except Exception as exc:
            self.get_logger().error(f"Failed to encode/write UART JSON payload {payload}: {exc}")
            return False

    def _poll_serial(self) -> None:
        if self._serial is None:
            return

        try:
            waiting = self._serial.in_waiting
            if waiting <= 0:
                return
            chunk = self._serial.read(waiting)
        except SerialException as exc:
            self.get_logger().error(f"UART read failed: {exc}")
            self._serial = None
            return
        except Exception as exc:
            self.get_logger().error(f"Unexpected UART read failure: {exc}")
            return

        if not chunk:
            return

        self._read_buffer.extend(chunk)
        while True:
            newline_index = self._read_buffer.find(b"\n")
            if newline_index < 0:
                return
            raw_line = bytes(self._read_buffer[:newline_index]).strip()
            del self._read_buffer[: newline_index + 1]
            if not raw_line:
                continue
            self._handle_line(raw_line)

    def _handle_line(self, raw_line: bytes) -> None:
        try:
            payload = json.loads(raw_line.decode("utf-8"))
        except Exception as exc:
            self.get_logger().warn(f"Ignoring invalid UART JSON line {raw_line!r}: {exc}")
            return

        message_type = payload.get("message_type")
        message = payload.get("message")

        if message_type != "currently_dispensing":
            self.get_logger().warn(f"Ignoring unsupported UART message type {message_type!r}")
            return

        if not isinstance(message, bool):
            self.get_logger().warn(
                f"Ignoring currently_dispensing payload with non-bool message {message!r}"
            )
            return

        self._publish_currently_dispensing(message)

    def destroy_node(self) -> bool:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UartDispenseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
