from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt8

try:
    # pyserial provides direct access to Linux serial devices such as /dev/ttyTHS1.
    import serial
    from serial import SerialException
except Exception:  # pragma: no cover - dependency availability is environment-specific
    serial = None
    SerialException = Exception


class UartDispenseBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_uart_dispense_bridge")

        # ROS parameters let the same node run against different UART ports/topics
        # without changing code.
        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("flavor_selection_topic", "/flavor_selection")
        self.declare_parameter("currently_dispensing_topic", "/currently_dispensing")
        self.declare_parameter("dispense_empty_topic", "/dispense_empty")
        self.declare_parameter("uart_event_topic", "/dispense_uart_event")
        self.declare_parameter("movement_gate_topic", "/return_home_trigger")
        self.declare_parameter("return_home_trigger_topic", "")
        self.declare_parameter("poll_hz", 50.0)
        self.declare_parameter("write_timeout_sec", 0.2)
        self.declare_parameter("read_timeout_sec", 0.0)
        self.declare_parameter("initial_currently_dispensing", False)
        self.declare_parameter("initial_dispense_empty", False)

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.flavor_selection_topic = str(self.get_parameter("flavor_selection_topic").value)
        self.currently_dispensing_topic = str(
            self.get_parameter("currently_dispensing_topic").value
        )
        self.dispense_empty_topic = str(self.get_parameter("dispense_empty_topic").value)
        self.uart_event_topic = str(self.get_parameter("uart_event_topic").value)
        self.movement_gate_topic = str(
            self.get_parameter("movement_gate_topic").value
            or self.get_parameter("return_home_trigger_topic").value
            or "/return_home_trigger"
        )
        poll_hz = max(1.0, float(self.get_parameter("poll_hz").value))
        self.write_timeout_sec = float(self.get_parameter("write_timeout_sec").value)
        self.read_timeout_sec = float(self.get_parameter("read_timeout_sec").value)

        # Transient local QoS makes the latest state available to late-joining
        # subscribers such as the web UI.
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
        self.dispense_empty_pub = self.create_publisher(
            Bool,
            self.dispense_empty_topic,
            qos,
        )
        self.movement_gate_pub = self.create_publisher(
            Bool,
            self.movement_gate_topic,
            qos,
        )
        self.uart_event_pub = self.create_publisher(String, self.uart_event_topic, 10)
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
        self._dispense_empty = bool(self.get_parameter("initial_dispense_empty").value)
        self._last_flavor_selection: Optional[int] = None

        self._connect_serial()
        self._publish_currently_dispensing(self._currently_dispensing, force=True)
        self._publish_dispense_empty(self._dispense_empty, force=True)
        self.create_timer(1.0 / poll_hz, self._poll_serial)
        self.get_logger().info(
            f"UART dispense bridge listening on {self.port} at {self.baudrate} baud; "
            f"publishing dispensing state to {self.currently_dispensing_topic} and "
            f"empty state to {self.dispense_empty_topic}; reading flavor commands from "
            f"{self.flavor_selection_topic}"
        )

    def _connect_serial(self) -> None:
        """Open the configured UART device and keep the handle for later reads/writes."""
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
        """Publish the current dispensing state only when it changes unless forced."""
        if not force and self._currently_dispensing == state:
            return

        self._currently_dispensing = state
        msg = Bool()
        msg.data = state
        self.currently_dispensing_pub.publish(msg)
        self.get_logger().info(f"currently_dispensing -> {'true' if state else 'false'}")

    def _publish_dispense_empty(self, state: bool, *, force: bool = False) -> None:
        """Publish whether the dispenser is empty, again suppressing duplicate state."""
        if not force and self._dispense_empty == state:
            return

        self._dispense_empty = state
        msg = Bool()
        msg.data = state
        self.dispense_empty_pub.publish(msg)
        self.get_logger().info(f"dispense_empty -> {'true' if state else 'false'}")

    def _publish_uart_event(self, text: str) -> None:
        """Forward raw UART status text onto a ROS topic for debugging and UI display."""
        msg = String()
        msg.data = text
        self.uart_event_pub.publish(msg)

    def _publish_movement_gate_open(self) -> None:
        """Allow motion again when the dispenser reports EMPTY and service is complete."""
        msg = Bool()
        msg.data = True
        self.movement_gate_pub.publish(msg)
        self.get_logger().warn("Published movement gate OPEN after EMPTY status")

    def _on_flavor_selection(self, msg: UInt8) -> None:
        """Translate ROS flavor ids 1/2/3 into the ESP protocol bytes A/B/C."""
        selection = int(msg.data)
        flavor_code = {1: "A", 2: "B", 3: "C"}.get(selection)
        if flavor_code is None:
            self.get_logger().warn(f"Ignoring unsupported flavor selection value {selection}")
            return

        if self._write_text(flavor_code):
            if self._last_flavor_selection != selection:
                self.get_logger().info(f"Sent flavor selection {selection} as UART '{flavor_code}'")
                self._last_flavor_selection = selection

    def _write_text(self, payload: str) -> bool:
        """Write a short ASCII command to the ESP over UART."""
        if self._serial is None:
            self.get_logger().error(f"UART port {self.port} is not open; dropping {payload!r}")
            return False

        try:
            encoded = payload.encode("utf-8")
            self._serial.write(encoded)
            self._serial.flush()
            return True
        except SerialException as exc:
            self.get_logger().error(f"UART write failed: {exc}")
            self._serial = None
            return False
        except Exception as exc:
            self.get_logger().error(f"Failed to write UART payload {payload!r}: {exc}")
            return False

    def _poll_serial(self) -> None:
        """Read any pending UART bytes and split them into newline-terminated messages."""
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
        """Parse one UART status line from the ESP and update ROS state topics."""
        try:
            text = raw_line.decode("utf-8", errors="replace").strip()
        except Exception as exc:
            self.get_logger().warn(f"Ignoring invalid UART line {raw_line!r}: {exc}")
            return

        if not text:
            return

        self._publish_uart_event(text)
        upper_text = text.upper()

        if "EMPTY" in upper_text:
            self._publish_currently_dispensing(False)
            self._publish_dispense_empty(True)
            self._publish_movement_gate_open()
            return

        if "CUP" in upper_text:
            self._publish_currently_dispensing("REMOVED" not in upper_text)
            return

        if "REMOVED" in upper_text:
            self._publish_currently_dispensing(False)

    def destroy_node(self) -> bool:
        """Close the UART device before letting the ROS node shut down."""
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    """ROS entrypoint: initialize rclpy, run the node, then clean up resources."""
    rclpy.init(args=args)
    node = UartDispenseBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
