import math
import os
import struct
from typing import Final

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, SportModeState_


JOINT_NAMES: Final[list[str]] = [
    "lf_hip_joint",
    "lf_upper_leg_joint",
    "lf_lower_leg_joint",
    "rf_hip_joint",
    "rf_upper_leg_joint",
    "rf_lower_leg_joint",
    "lh_hip_joint",
    "lh_upper_leg_joint",
    "lh_lower_leg_joint",
    "rh_hip_joint",
    "rh_upper_leg_joint",
    "rh_lower_leg_joint",
]

MOTOR_INDEX_ORDER: Final[list[int]] = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
MOTOR_ORDER_TO_JOINT_ORDER: Final[list[int]] = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
STAND_POSE_MOTOR_ORDER: Final[list[float]] = [
    0.00571868,
    0.608813,
    -1.21763,
    -0.00571868,
    0.608813,
    -1.21763,
    0.00571868,
    0.608813,
    -1.21763,
    -0.00571868,
    0.608813,
    -1.21763,
]
LEG_PHASE_OFFSETS: Final[list[float]] = [0.0, 0.5, 0.75, 0.25]  # FR, FL, RR, RL
RIGHT_LEG_SIGNS: Final[list[float]] = [1.0, -1.0, 1.0, -1.0]
LEG_JOINT_SLICE: Final[list[tuple[int, int]]] = [(0, 3), (3, 6), (6, 9), (9, 12)]
LOW_CMD_PACK_FMT: Final[str] = "<4B4IH2x" + "B3x5f3I" * 20 + "4B" + "55Bx2I"


def _crc_py(data: list[int]) -> int:
    crc = 0xFFFFFFFF
    polynomial = 0x04C11DB7

    for current in data:
        bit = 1 << 31
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) & 0xFFFFFFFF) ^ polynomial
            else:
                crc = (crc << 1) & 0xFFFFFFFF

            if current & bit:
                crc ^= polynomial
            bit >>= 1

    return crc


def _pack_low_cmd_for_crc(msg: LowCmd_) -> list[int]:
    raw = []
    raw.extend(msg.head)
    raw.append(msg.level_flag)
    raw.append(msg.frame_reserve)
    raw.extend(msg.sn)
    raw.extend(msg.version)
    raw.append(msg.bandwidth)

    for index in range(20):
        raw.append(msg.motor_cmd[index].mode)
        raw.append(msg.motor_cmd[index].q)
        raw.append(msg.motor_cmd[index].dq)
        raw.append(msg.motor_cmd[index].tau)
        raw.append(msg.motor_cmd[index].kp)
        raw.append(msg.motor_cmd[index].kd)
        raw.extend(msg.motor_cmd[index].reserve)

    raw.append(msg.bms_cmd.off)
    raw.extend(msg.bms_cmd.reserve)
    raw.extend(msg.wireless_remote)
    raw.extend(msg.led)
    raw.extend(msg.fan)
    raw.append(msg.gpio)
    raw.append(msg.reserve)
    raw.append(msg.crc)

    packed = struct.pack(LOW_CMD_PACK_FMT, *raw)
    words = []
    word_count = (len(packed) >> 2) - 1
    for index in range(word_count):
        offset = index * 4
        words.append(
            (packed[offset + 3] << 24)
            | (packed[offset + 2] << 16)
            | (packed[offset + 1] << 8)
            | packed[offset]
        )
    return words


class Go2MujocoBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_mujoco_bridge_node")

        self.declare_parameter("domain_id", 1)
        self.declare_parameter("interface", "lo")
        self.declare_parameter("low_state_topic", "rt/lowstate")
        self.declare_parameter("sport_state_topic", "rt/sportmodestate")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("low_cmd_topic", "rt/lowcmd")
        self.declare_parameter("control_rate_hz", 250.0)
        self.declare_parameter("cmd_vel_timeout_sec", 0.35)
        self.declare_parameter("max_linear_velocity", 0.35)
        self.declare_parameter("max_angular_velocity", 0.9)

        domain_id = int(self.get_parameter("domain_id").value)
        interface = str(self.get_parameter("interface").value)
        low_state_topic = str(self.get_parameter("low_state_topic").value)
        sport_state_topic = str(self.get_parameter("sport_state_topic").value)
        low_cmd_topic = str(self.get_parameter("low_cmd_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.publish_tf_enabled = bool(self.get_parameter("publish_tf").value)
        self.publish_odom_enabled = bool(self.get_parameter("publish_odom").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.cmd_vel_timeout_sec = float(self.get_parameter("cmd_vel_timeout_sec").value)
        self.max_linear_velocity = float(self.get_parameter("max_linear_velocity").value)
        self.max_angular_velocity = float(self.get_parameter("max_angular_velocity").value)

        ChannelFactoryInitialize(domain_id, interface)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.imu_publisher = self.create_publisher(Imu, "/imu/data", 10)
        self.joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self._low_cmd_publisher = ChannelPublisher(low_cmd_topic, LowCmd_)
        self._low_cmd_publisher.Init()
        self._low_cmd = unitree_go_msg_dds__LowCmd_()
        self._initialize_low_cmd()
        self._last_cmd_vel = Twist()
        self._last_cmd_time = self.get_clock().now()
        self._last_motion_log_time = self.get_clock().now()
        self._gait_phase = 0.0
        self._latest_orientation = (0.0, 0.0, 0.0, 1.0)
        self._control_timer = self.create_timer(1.0 / self.control_rate_hz, self._control_loop)

        self._low_state_sub = ChannelSubscriber(low_state_topic, LowState_)
        self._low_state_sub.Init(self._on_low_state, 10)
        self._sport_state_sub = ChannelSubscriber(sport_state_topic, SportModeState_)
        self._sport_state_sub.Init(self._on_sport_state, 10)

        self.get_logger().info(
            f"MuJoCo bridge listening on domain_id={domain_id} interface={interface} "
            f"topics {sport_state_topic}, {low_state_topic}, and {low_cmd_topic}"
        )

    def _on_sport_state(self, msg: SportModeState_) -> None:
        stamp = self.get_clock().now().to_msg()

        if self.publish_odom_enabled:
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.pose.pose.position.x = float(msg.position[0])
            odom.pose.pose.position.y = float(msg.position[1])
            odom.pose.pose.position.z = float(msg.position[2])
            odom.pose.pose.orientation.x = self._latest_orientation[0]
            odom.pose.pose.orientation.y = self._latest_orientation[1]
            odom.pose.pose.orientation.z = self._latest_orientation[2]
            odom.pose.pose.orientation.w = self._latest_orientation[3]
            odom.twist.twist.linear.x = float(msg.velocity[0])
            odom.twist.twist.linear.y = float(msg.velocity[1])
            odom.twist.twist.linear.z = float(msg.velocity[2])
            self.odom_publisher.publish(odom)

        if self.publish_tf_enabled:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = float(msg.position[0])
            transform.transform.translation.y = float(msg.position[1])
            transform.transform.translation.z = float(msg.position[2])
            transform.transform.rotation.x = self._latest_orientation[0]
            transform.transform.rotation.y = self._latest_orientation[1]
            transform.transform.rotation.z = self._latest_orientation[2]
            transform.transform.rotation.w = self._latest_orientation[3]
            self.tf_broadcaster.sendTransform(transform)

    def _on_low_state(self, msg: LowState_) -> None:
        stamp = self.get_clock().now().to_msg()

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = JOINT_NAMES
        joint_state.position = [float(msg.motor_state[index].q) for index in MOTOR_INDEX_ORDER]
        joint_state.velocity = [float(msg.motor_state[index].dq) for index in MOTOR_INDEX_ORDER]
        self.joint_state_publisher.publish(joint_state)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        imu.orientation.x = float(msg.imu_state.quaternion[0])
        imu.orientation.y = float(msg.imu_state.quaternion[1])
        imu.orientation.z = float(msg.imu_state.quaternion[2])
        imu.orientation.w = float(msg.imu_state.quaternion[3])
        self._latest_orientation = (
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
            imu.orientation.w,
        )
        imu.angular_velocity.x = float(msg.imu_state.gyroscope[0])
        imu.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu.angular_velocity.z = float(msg.imu_state.gyroscope[2])
        imu.linear_acceleration.x = float(msg.imu_state.accelerometer[0])
        imu.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu.linear_acceleration.z = float(msg.imu_state.accelerometer[2])
        self.imu_publisher.publish(imu)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_vel = msg
        self._last_cmd_time = self.get_clock().now()

    def _initialize_low_cmd(self) -> None:
        self._low_cmd.head[0] = 0xFE
        self._low_cmd.head[1] = 0xEF
        self._low_cmd.level_flag = 0xFF
        self._low_cmd.gpio = 0
        for index in range(20):
            self._low_cmd.motor_cmd[index].mode = 0x01
            self._low_cmd.motor_cmd[index].q = 0.0
            self._low_cmd.motor_cmd[index].kp = 0.0
            self._low_cmd.motor_cmd[index].dq = 0.0
            self._low_cmd.motor_cmd[index].kd = 0.0
            self._low_cmd.motor_cmd[index].tau = 0.0

    def _control_loop(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self._last_cmd_time).nanoseconds / 1e9
        cmd_active = elapsed <= self.cmd_vel_timeout_sec

        linear_x = float(self._last_cmd_vel.linear.x) if cmd_active else 0.0
        angular_z = float(self._last_cmd_vel.angular.z) if cmd_active else 0.0
        linear_norm = max(-1.0, min(1.0, linear_x / max(self.max_linear_velocity, 1e-6)))
        angular_norm = max(-1.0, min(1.0, angular_z / max(self.max_angular_velocity, 1e-6)))

        motion_scale = max(abs(linear_norm), abs(angular_norm))
        target_positions = STAND_POSE_MOTOR_ORDER.copy()

        if motion_scale > 0.05:
            dt = 1.0 / self.control_rate_hz
            gait_frequency = 0.55 + 0.35 * motion_scale
            self._gait_phase = (self._gait_phase + dt * gait_frequency) % 1.0
            step_sweep = 0.06 * linear_norm
            turn_sweep = 0.025 * angular_norm
            knee_lift = 0.10 * motion_scale
            thigh_lift = 0.05 * motion_scale
            hip_balance = 0.015 * angular_norm

            for leg_index, (start, end) in enumerate(LEG_JOINT_SLICE):
                phase = (self._gait_phase + LEG_PHASE_OFFSETS[leg_index]) % 1.0
                lateral_sign = RIGHT_LEG_SIGNS[leg_index]
                stride = step_sweep + lateral_sign * turn_sweep

                if phase < 0.25:
                    swing_progress = phase / 0.25
                    sweep = -0.5 + swing_progress
                    lift = math.sin(math.pi * swing_progress)
                else:
                    stance_progress = (phase - 0.25) / 0.75
                    sweep = 0.5 - stance_progress
                    lift = 0.0

                hip_target = STAND_POSE_MOTOR_ORDER[start] + lateral_sign * hip_balance
                thigh_target = STAND_POSE_MOTOR_ORDER[start + 1] + stride * sweep + thigh_lift * lift
                calf_target = STAND_POSE_MOTOR_ORDER[start + 2] - 1.4 * stride * sweep - knee_lift * lift

                target_positions[start] = hip_target
                target_positions[start + 1] = thigh_target
                target_positions[start + 2] = calf_target

            if (now - self._last_motion_log_time).nanoseconds > int(1.5e9):
                self.get_logger().info(
                    "Lowcmd gait active: "
                    f"linear_x={linear_x:.3f} angular_z={angular_z:.3f} "
                    f"phase={self._gait_phase:.2f}"
                )
                self._last_motion_log_time = now
        else:
            self._gait_phase = 0.0

        for motor_index in range(12):
            self._low_cmd.motor_cmd[motor_index].q = float(target_positions[motor_index])
            self._low_cmd.motor_cmd[motor_index].kp = 32.0
            self._low_cmd.motor_cmd[motor_index].dq = 0.0
            self._low_cmd.motor_cmd[motor_index].kd = 4.5
            self._low_cmd.motor_cmd[motor_index].tau = 0.0

        for motor_index in range(12, 20):
            self._low_cmd.motor_cmd[motor_index].q = 0.0
            self._low_cmd.motor_cmd[motor_index].kp = 0.0
            self._low_cmd.motor_cmd[motor_index].dq = 0.0
            self._low_cmd.motor_cmd[motor_index].kd = 0.0
            self._low_cmd.motor_cmd[motor_index].tau = 0.0

        self._low_cmd.crc = _crc_py(_pack_low_cmd_for_crc(self._low_cmd))
        self._low_cmd_publisher.Write(self._low_cmd)


def main(args=None) -> None:
    domain_id = int(os.getenv("UNITREE_MJ_DOMAIN_ID", "1"))
    interface = os.getenv("UNITREE_MJ_INTERFACE", "lo")
    ChannelFactoryInitialize(domain_id, interface)
    rclpy.init(args=args)
    node = Go2MujocoBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
