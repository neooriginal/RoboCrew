"""Lightweight wheel and arm helpers for the XLeRobot."""

from __future__ import annotations

import time
import json
from pathlib import Path
from typing import Dict, Mapping, Optional
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode


DEFAULT_BAUDRATE = 1_000_000
DEFAULT_SPEED = 10_000
LINEAR_MPS = 0.25
ANGULAR_DPS = 100.0

# Wheel motors: IDs 7, 8, 9 on right_arm_wheel bus
ACTION_MAP = {
    "up": {7: 1, 8: 0, 9: -1},
    "down": {7: -1, 8: 0, 9: 1},
    "left": {7: 1, 8: 1, 9: 1},
    "right": {7: -1, 8: -1, 9: -1},
}

# Head motors: IDs 7, 8 on left_arm_head bus
HEAD_SERVO_MAP = {"yaw": 7, "pitch": 8}

# Right arm motors: IDs 1-6 on SAME bus as wheels (right_arm_wheel)
ARM_SERVO_MAP = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

# Arm joint limits in degrees
ARM_LIMITS = {
    "shoulder_pan": (-90, 90),
    "shoulder_lift": (-90, 90),
    "elbow_flex": (-90, 90),
    "wrist_flex": (-90, 90),
    "wrist_roll": (-150, 150),
    "gripper": (0, 90),  # 0 = closed, 90 = open
}


class ServoControler:
    """Controller for wheels, head, and right arm.
    
    Motor layout:
    - right_arm_wheel_usb: Wheels (7,8,9) + Right Arm (1-6)
    - left_arm_head_usb: Head (7,8)
    """

    def __init__(
        self,
        right_arm_wheel_usb: str = None,
        left_arm_head_usb: str = None,
        *,
        speed: int = DEFAULT_SPEED,
        action_map: Optional[Mapping[str, Mapping[int, int]]] = None,
        enable_arm: bool = False,
        arm_calibration_path: str = None,
    ) -> None:
        self.right_arm_wheel_usb = right_arm_wheel_usb
        self.left_arm_head_usb = left_arm_head_usb
        self.speed = speed
        self.action_map = ACTION_MAP if action_map is None else action_map
        self._wheel_ids = tuple(sorted(next(iter(self.action_map.values())).keys()))
        self._head_ids = tuple(sorted(HEAD_SERVO_MAP.values()))
        self._arm_ids = tuple(sorted(ARM_SERVO_MAP.values()))
        
        self._arm_positions = {}
        self._arm_enabled = False
        self.wheel_bus = None
        self.head_bus = None

        # Initialize wheel/arm bus (they share the same port!)
        if right_arm_wheel_usb:
            # Build motor dict - always include wheels
            motors = {
                7: Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                8: Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                9: Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
            }
            
            calibration = None
            
            # Add arm motors if enabled
            if enable_arm:
                # Load arm calibration if provided
                arm_calibration = {}
                if arm_calibration_path:
                    cal_path = Path(arm_calibration_path)
                    if cal_path.exists():
                        with open(cal_path) as f:
                            cal_data = json.load(f)
                        for joint_name, cal in cal_data.items():
                            motor_id = cal["id"]
                            arm_calibration[motor_id] = MotorCalibration(
                                id=motor_id,
                                drive_mode=cal.get("drive_mode", 0),
                                homing_offset=cal.get("homing_offset", 0),
                                range_min=cal.get("range_min", 0),
                                range_max=cal.get("range_max", 4095),
                            )
                
                # Add arm motors (IDs 1-6)
                for joint_name, motor_id in ARM_SERVO_MAP.items():
                    motors[motor_id] = Motor(motor_id, "sts3215", MotorNormMode.DEGREES)
                
                if arm_calibration:
                    calibration = arm_calibration
            
            self.wheel_bus = FeetechMotorsBus(
                port=right_arm_wheel_usb,
                motors=motors,
                calibration=calibration,
            )
            self.wheel_bus.connect()
            self.apply_wheel_modes()
            
            if enable_arm:
                self._apply_arm_modes()
                self._arm_enabled = True
                self._arm_positions = self.get_arm_position()
        
        # Head motors on separate bus (left_arm_head)
        head_calibration = {
            7: MotorCalibration(
                id=7,
                drive_mode=0,
                homing_offset=0,
                range_min=0,
                range_max=4095,
            ),
            8: MotorCalibration(
                id=8,
                drive_mode=0,
                homing_offset=0,
                range_min=0,
                range_max=4095,
            ),
        }
        
        if left_arm_head_usb:
            self.head_bus = FeetechMotorsBus(
                port=left_arm_head_usb,
                motors={
                    HEAD_SERVO_MAP["yaw"]: Motor(HEAD_SERVO_MAP["yaw"], "sts3215", MotorNormMode.DEGREES),
                    HEAD_SERVO_MAP["pitch"]: Motor(HEAD_SERVO_MAP["pitch"], "sts3215", MotorNormMode.DEGREES),
                },
                calibration=head_calibration,
            )
            self.head_bus.connect()
            self.apply_head_modes()
            self._head_positions = self.get_head_position()
            for sid in self._head_ids:
                self._head_positions.setdefault(sid, 2048)

    @property
    def arm_enabled(self) -> bool:
        return self._arm_enabled

    # ============== Wheel Control ==============

    def _wheels_write(self, action: str) -> Dict[int, int]:
        multipliers = self.action_map[action.lower()]
        payload = {wid: self.speed * factor for wid, factor in multipliers.items()}
        self.wheel_bus.sync_write("Goal_Velocity", payload)
        return payload

    def _wheels_stop(self) -> Dict[int, int]:
        payload = {wid: 0 for wid in self._wheel_ids}
        self.wheel_bus.sync_write("Goal_Velocity", payload)
        return payload

    def _wheels_run(self, action: str, duration: float) -> Dict[int, int]:
        if duration <= 0:
            return {}
        payload = self._wheels_write(action)
        time.sleep(duration)
        self._wheels_stop()
        return payload

    def go_forward(self, meters: float) -> Dict[int, int]:
        return self._wheels_run("up", float(meters) / LINEAR_MPS)

    def go_backward(self, meters: float) -> Dict[int, int]:
        return self._wheels_run("down", float(meters) / LINEAR_MPS)

    def turn_left(self, degrees: float) -> Dict[int, int]:
        return self._wheels_run("left", float(degrees) / ANGULAR_DPS)

    def turn_right(self, degrees: float) -> Dict[int, int]:
        return self._wheels_run("right", float(degrees) / ANGULAR_DPS)

    def apply_wheel_modes(self) -> None:
        for wid in self._wheel_ids:
            self.wheel_bus.write("Operating_Mode", wid, OperatingMode.VELOCITY.value)
        self.wheel_bus.enable_torque()

    # ============== Head Control ==============

    def apply_head_modes(self) -> None:
        for sid in self._head_ids:
            self.head_bus.write("Operating_Mode", sid, OperatingMode.POSITION.value)
        self.head_bus.enable_torque()

    def turn_head_yaw(self, degrees: float) -> Dict[int, float]:
        payload = {HEAD_SERVO_MAP["yaw"]: float(degrees)}
        self.head_bus.sync_write("Goal_Position", payload)
        self._head_positions.update(payload)
        return payload

    def turn_head_pitch(self, degrees: float) -> Dict[int, float]:
        payload = {HEAD_SERVO_MAP["pitch"]: float(degrees)}
        self.head_bus.sync_write("Goal_Position", payload)
        self._head_positions.update(payload)
        return payload

    def get_head_position(self) -> Dict[int, float]:
        return self.head_bus.sync_read("Present_Position", list(self._head_ids))
    
    def turn_head_to_vla_position(self, pitch_deg=45) -> str:
        self.turn_head_pitch(pitch_deg)
        self.turn_head_yaw(0)

    def reset_head_position(self) -> str:
        self.turn_head_pitch(35)
        self.turn_head_yaw(0)

    # ============== Arm Control (uses wheel_bus, IDs 1-6) ==============

    def _apply_arm_modes(self) -> None:
        """Set arm motors to position mode and enable torque."""
        for motor_id in self._arm_ids:
            self.wheel_bus.write("Operating_Mode", motor_id, OperatingMode.POSITION.value)
        # Torque is already enabled for wheels, but arm motors need it too
        # The enable_torque call affects all motors on the bus

    def get_arm_position(self) -> Dict[str, float]:
        """Read current arm joint positions in degrees."""
        if not self._arm_enabled:
            return {}
        raw = self.wheel_bus.sync_read("Present_Position", list(self._arm_ids))
        result = {}
        for joint_name, motor_id in ARM_SERVO_MAP.items():
            result[joint_name] = raw.get(motor_id, 0.0)
        return result

    def set_arm_position(self, positions: Dict[str, float]) -> Dict[str, float]:
        """Set arm joint positions. Applies limits for safety."""
        if not self._arm_enabled:
            return {}
        
        payload = {}
        for joint_name, angle in positions.items():
            if joint_name in ARM_SERVO_MAP:
                motor_id = ARM_SERVO_MAP[joint_name]
                limits = ARM_LIMITS.get(joint_name, (-180, 180))
                clamped = max(limits[0], min(limits[1], float(angle)))
                payload[motor_id] = clamped
                self._arm_positions[joint_name] = clamped
        
        if payload:
            self.wheel_bus.sync_write("Goal_Position", payload)
        return self._arm_positions.copy()

    def set_arm_joint(self, joint_name: str, angle: float) -> float:
        """Set a single arm joint position."""
        if joint_name not in ARM_SERVO_MAP:
            return 0.0
        result = self.set_arm_position({joint_name: angle})
        return result.get(joint_name, 0.0)

    def set_gripper(self, closed: bool) -> float:
        """Set gripper state. closed=True closes gripper, closed=False opens it."""
        angle = 0.0 if closed else 90.0
        return self.set_arm_joint("gripper", angle)

    # ============== Cleanup ==============

    def disconnect(self) -> None:
        self._wheels_stop()
        if self.wheel_bus:
            self.wheel_bus.disconnect()
        if self.head_bus:
            self.head_bus.disconnect()

    def __del__(self) -> None:
        if hasattr(self, "wheel_bus") and self.wheel_bus and self.wheel_bus.is_connected:
            self.disconnect()
