from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8

from go2_navigation.gpio_platform import import_jetson_gpio

GPIO, GPIO_IMPORT_ERROR, inferred_jetson_model = import_jetson_gpio()


class GpioFlavorSelection(Node):
    def __init__(self) -> None:
        super().__init__("go2_gpio_flavor_selection")

        self.declare_parameter("topic", "/flavor_selection")
        self.declare_parameter("pin_mode", "BOARD")
        self.declare_parameter("gpio12_pin_number", 15)
        self.declare_parameter("gpio11_pin_number", 31)
        self.declare_parameter("default_selection", 0)

        self.topic = str(self.get_parameter("topic").value)
        self.pin_mode = str(self.get_parameter("pin_mode").value).upper()
        self.gpio12_pin_number = int(self.get_parameter("gpio12_pin_number").value)
        self.gpio11_pin_number = int(self.get_parameter("gpio11_pin_number").value)
        default_selection = int(self.get_parameter("default_selection").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._gpio_ready = False
        self._current_selection: Optional[int] = None

        if GPIO is None:
            if GPIO_IMPORT_ERROR is None:
                reason = "Jetson.GPIO is not installed"
            else:
                reason = f"Jetson.GPIO is unavailable: {GPIO_IMPORT_ERROR}"
            if inferred_jetson_model is not None:
                reason += f" (auto-detected model {inferred_jetson_model})"
            self.get_logger().error(f"{reason}; flavor GPIO outputs are disabled")
        else:
            try:
                self._setup_gpio()
                self._apply_selection(default_selection)
            except Exception as exc:
                self.get_logger().error(f"Failed to initialize flavor GPIO outputs: {exc}")

        self.create_subscription(UInt8, self.topic, self._on_selection, qos)
        self.get_logger().info(
            f"Listening for flavor selection on {self.topic}; pin {self.gpio12_pin_number}=gpio12, "
            f"pin {self.gpio11_pin_number}=gpio11"
        )

    def _setup_gpio(self) -> None:
        mode = getattr(GPIO, self.pin_mode, None)
        if mode is None:
            raise ValueError(f"Unsupported GPIO mode '{self.pin_mode}'")

        GPIO.setwarnings(False)
        GPIO.setmode(mode)
        GPIO.setup(self.gpio12_pin_number, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.gpio11_pin_number, GPIO.OUT, initial=GPIO.LOW)
        self._gpio_ready = True

    def _on_selection(self, msg: UInt8) -> None:
        selection = int(msg.data)
        if selection not in (0, 1, 2, 3):
            self.get_logger().warn(f"Ignoring unsupported flavor selection value {selection}")
            return
        self._apply_selection(selection)

    def _apply_selection(self, selection: int) -> None:
        if not self._gpio_ready:
            self._current_selection = selection
            return

        gpio12_high = bool(selection & 0b10)
        gpio11_high = bool(selection & 0b01)

        GPIO.output(self.gpio12_pin_number, GPIO.HIGH if gpio12_high else GPIO.LOW)
        GPIO.output(self.gpio11_pin_number, GPIO.HIGH if gpio11_high else GPIO.LOW)

        if self._current_selection != selection:
            self.get_logger().info(
                f"Flavor selection set to {selection:02b} "
                f"(gpio12={'HIGH' if gpio12_high else 'LOW'}, gpio11={'HIGH' if gpio11_high else 'LOW'})"
            )
            self._current_selection = selection

    def destroy_node(self) -> bool:
        if GPIO is not None and self._gpio_ready:
            GPIO.cleanup(self.gpio12_pin_number)
            GPIO.cleanup(self.gpio11_pin_number)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpioFlavorSelection()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
