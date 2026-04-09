from __future__ import annotations

import os
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def _infer_jetson_model_name() -> Optional[str]:
    if os.environ.get("JETSON_MODEL_NAME"):
        return None

    try:
        with open("/proc/device-tree/compatible", "rb") as compatible_file:
            compatible = compatible_file.read().decode("utf-8", errors="ignore")
    except OSError:
        return None

    entries = [entry for entry in compatible.split("\x00") if entry]
    normalized_entries = {entry.removesuffix("-super") for entry in entries}

    orin_nano_compatibles = {
        "nvidia,p3509-0000+p3767-0003",
        "nvidia,p3768-0000+p3767-0003",
        "nvidia,p3509-0000+p3767-0004",
        "nvidia,p3768-0000+p3767-0004",
        "nvidia,p3509-0000+p3767-0005",
        "nvidia,p3768-0000+p3767-0005",
    }
    if normalized_entries & orin_nano_compatibles:
        return "JETSON_ORIN_NANO"
    return None


inferred_jetson_model = _infer_jetson_model_name()
if inferred_jetson_model is not None:
    os.environ["JETSON_MODEL_NAME"] = inferred_jetson_model

try:
    import Jetson.GPIO as GPIO
    GPIO_IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover - hardware-specific dependency
    GPIO = None
    GPIO_IMPORT_ERROR = exc


class GpioReturnHomePublisher(Node):
    def __init__(self) -> None:
        super().__init__("go2_gpio_return_home_publisher")

        self.declare_parameter("topic", "/return_home_trigger")
        self.declare_parameter("pin_number", 7)
        self.declare_parameter("pin_mode", "BOARD")
        self.declare_parameter("pull", "DOWN")
        self.declare_parameter("poll_hz", 5.0)

        topic = str(self.get_parameter("topic").value)
        self.pin_number = int(self.get_parameter("pin_number").value)
        self.pin_mode = str(self.get_parameter("pin_mode").value).upper()
        self.pull = str(self.get_parameter("pull").value).upper()
        poll_hz = max(0.5, float(self.get_parameter("poll_hz").value))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(Bool, topic, qos)
        self.last_state: Optional[bool] = None
        self._gpio_ready = False

        if GPIO is None:
            if GPIO_IMPORT_ERROR is None:
                reason = "Jetson.GPIO is not installed"
            else:
                reason = f"Jetson.GPIO is unavailable: {GPIO_IMPORT_ERROR}"
            if inferred_jetson_model is not None:
                reason += f" (auto-detected model {inferred_jetson_model})"
            self.get_logger().error(f"{reason}; GPIO return-home publishing is disabled")
            self._publish_state(False)
            return

        try:
            self._setup_gpio()
        except Exception as exc:
            self.get_logger().error(f"Failed to initialize GPIO input: {exc}")
            self._publish_state(False)
            return
        self.timer = self.create_timer(1.0 / poll_hz, self._poll_gpio)
        self._poll_gpio()
        self.get_logger().info(
            f"Publishing GPIO return-home state from {self.pin_mode} pin {self.pin_number} to {topic}"
        )

    def _setup_gpio(self) -> None:
        mode = getattr(GPIO, self.pin_mode, None)
        if mode is None:
            raise ValueError(f"Unsupported GPIO mode '{self.pin_mode}'")

        pull = getattr(GPIO, f"PUD_{self.pull}", None)
        if pull is None:
            raise ValueError(f"Unsupported GPIO pull '{self.pull}'")

        GPIO.setwarnings(False)
        GPIO.setmode(mode)
        GPIO.setup(self.pin_number, GPIO.IN, pull_up_down=pull)
        self._gpio_ready = True

    def _poll_gpio(self) -> None:
        if not self._gpio_ready:
            return
        state = bool(GPIO.input(self.pin_number))
        self._publish_state(state)

    def _publish_state(self, state: bool) -> None:
        msg = Bool()
        msg.data = state
        self.publisher.publish(msg)
        if self.last_state != state:
            level = "HIGH" if state else "LOW"
            self.get_logger().info(f"Return-home GPIO is now {level}")
            self.last_state = state

    def destroy_node(self) -> bool:
        if GPIO is not None and self._gpio_ready:
            GPIO.cleanup(self.pin_number)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpioReturnHomePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
