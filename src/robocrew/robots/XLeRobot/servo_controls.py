"""Lightweight wheel helpers for the XLeRobot."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Dict, Mapping, Optional
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode


DEFAULT_BAUDRATE = 1_000_000
DEFAULT_SPEED = 10_000
LINEAR_MPS = 0.25
ANGULAR_DPS = 100.0

ACTION_MAP = {
    "up": {7: 1, 8: 0, 9: -1},
    "down": {7: -1, 8: 0, 9: 1},
    "left": {7: 1, 8: 1, 9: 1},
    "right": {7: -1, 8: -1, 9: -1},
}

HEAD_SERVO_MAP = {"yaw": 7, "pitch": 8}

# Arm servo mapping (right arm)
ARM_SERVO_MAP = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

# Default arm joint limits (in degrees, approximate)
ARM_LIMITS = {
    "shoulder_pan": (-90, 90),
    "shoulder_lift": (-90, 90),
    "elbow_flex": (-90, 90),
    "wrist_flex": (-90, 90),
    "wrist_roll": (-180, 180),
    "gripper": (0, 100),  # 0 = closed, 100 = open
}


class ServoControler:
    """Minimal wheel controller that keeps only basic movement helpers."""

    def __init__(
        self,
        right_arm_wheel_usb: str = None,
        left_arm_head_usb: str = None,
        *,
        speed: int = DEFAULT_SPEED,
        action_map: Optional[Mapping[str, Mapping[int, int]]] = None,
        calibration_path: Optional[str] = None,
    ) -> None:
        self.right_arm_wheel_usb = right_arm_wheel_usb
        self.left_arm_head_usb = left_arm_head_usb
        self.speed = speed
        self.action_map = ACTION_MAP if action_map is None else action_map
        self._wheel_ids = tuple(sorted(next(iter(self.action_map.values())).keys()))
        self._head_ids = tuple(sorted(HEAD_SERVO_MAP.values()))
        self._arm_ids = tuple(sorted(ARM_SERVO_MAP.values()))
        
        # Load arm calibration if available
        self._arm_calibration = {}
        if calibration_path:
            self._load_arm_calibration(calibration_path)

        # Initialize FeetechMotorsBus with wheels AND arm motors
        if right_arm_wheel_usb:
            # Combine wheel motors (7,8,9) and arm motors (1-6)
            motors = {
                # Wheel motors
                7: Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                8: Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                9: Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
                # Arm motors
                1: Motor(1, "sts3215", MotorNormMode.DEGREES),
                2: Motor(2, "sts3215", MotorNormMode.DEGREES),
                3: Motor(3, "sts3215", MotorNormMode.DEGREES),
                4: Motor(4, "sts3215", MotorNormMode.DEGREES),
                5: Motor(5, "sts3215", MotorNormMode.DEGREES),
                6: Motor(6, "sts3215", MotorNormMode.DEGREES),
            }
            
            # Build calibration for arm motors if available
            calibration = {}
            for joint_name, motor_id in ARM_SERVO_MAP.items():
                if joint_name in self._arm_calibration:
                    cal = self._arm_calibration[joint_name]
                    calibration[motor_id] = MotorCalibration(
                        id=motor_id,
                        drive_mode=cal.get("drive_mode", 0),
                        homing_offset=cal.get("homing_offset", 0),
                        range_min=cal.get("range_min", 0),
                        range_max=cal.get("range_max", 4095),
                    )
            
            self.wheel_bus = FeetechMotorsBus(
                port=right_arm_wheel_usb,
                motors=motors,
                calibration=calibration if calibration else None,
            )
            self.wheel_bus.connect()
            self.apply_wheel_modes()
            self.apply_arm_modes()
            
            # Read initial arm positions
            self._arm_positions = self.get_arm_position()
        
        # Create basic calibration for head motors
        # STS3215 motors have 4096 positions (0-4095) which typically map to ~360 degrees
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
        
        # Initialize FeetechMotorsBus for head motors
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

    def _load_arm_calibration(self, path: str) -> None:
        """Load arm calibration from JSON file."""
        try:
            cal_path = Path(path)
            if cal_path.exists():
                with open(cal_path, "r") as f:
                    self._arm_calibration = json.load(f)
                print(f"[ARM] Loaded calibration from {path}")
        except Exception as e:
            print(f"[ARM] Could not load calibration: {e}")

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
    
    def turn_head_to_vla_position(self, pitch_deg=45) -> str:
        self.turn_head_pitch(pitch_deg)
        self.turn_head_yaw(0)

    def reset_head_position(self) -> str:
        self.turn_head_pitch(35)
        self.turn_head_yaw(0)

    def apply_wheel_modes(self) -> None:
        for wid in self._wheel_ids:
            self.wheel_bus.write("Operating_Mode", wid, OperatingMode.VELOCITY.value)

        self.wheel_bus.enable_torque()

    def apply_head_modes(self) -> None:
        for id in self._head_ids:
            self.head_bus.write("Operating_Mode", id, OperatingMode.POSITION.value)

        self.head_bus.enable_torque()

    def apply_arm_modes(self) -> None:
        """Set arm servos to position control mode."""
        for aid in self._arm_ids:
            self.wheel_bus.write("Operating_Mode", aid, OperatingMode.POSITION.value)
        # Enable torque for arm motors
        self.wheel_bus.sync_write("Torque_Enable", {aid: 1 for aid in self._arm_ids})

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

    # ==================== ARM CONTROL METHODS ====================
    
    def get_arm_position(self) -> Dict[str, float]:
        """Get current positions of all arm joints in degrees."""
        try:
            raw_positions = self.wheel_bus.sync_read("Present_Position", list(self._arm_ids))
            # Convert to named positions
            positions = {}
            for joint_name, motor_id in ARM_SERVO_MAP.items():
                positions[joint_name] = round(raw_positions.get(motor_id, 0), 1)
            return positions
        except Exception as e:
            print(f"[ARM READ ERROR] {e}")
            return {name: 0 for name in ARM_SERVO_MAP.keys()}

    def set_arm_joint(self, joint: str, degrees: float) -> Dict[str, float]:
        """Set a single arm joint to the specified position in degrees."""
        if joint not in ARM_SERVO_MAP:
            raise ValueError(f"Unknown joint: {joint}. Valid: {list(ARM_SERVO_MAP.keys())}")
        
        motor_id = ARM_SERVO_MAP[joint]
        
        # Clamp to limits
        limits = ARM_LIMITS.get(joint, (-180, 180))
        degrees = max(limits[0], min(limits[1], float(degrees)))
        
        payload = {motor_id: degrees}
        self.wheel_bus.sync_write("Goal_Position", payload)
        
        # Update cached position
        if hasattr(self, '_arm_positions'):
            self._arm_positions[joint] = degrees
        
        return {joint: degrees}

    def set_arm_position(self, positions: Dict[str, float]) -> Dict[str, float]:
        """Set multiple arm joints at once."""
        payload = {}
        result = {}
        
        for joint, degrees in positions.items():
            if joint not in ARM_SERVO_MAP:
                continue
            
            motor_id = ARM_SERVO_MAP[joint]
            limits = ARM_LIMITS.get(joint, (-180, 180))
            clamped = max(limits[0], min(limits[1], float(degrees)))
            
            payload[motor_id] = clamped
            result[joint] = clamped
        
        if payload:
            self.wheel_bus.sync_write("Goal_Position", payload)
            
            # Update cached positions
            if hasattr(self, '_arm_positions'):
                self._arm_positions.update(result)
        
        return result

    def set_gripper(self, open_percent: float) -> Dict[str, float]:
        """
        Set gripper position.
        open_percent: 0 = fully closed, 100 = fully open
        """
        # Clamp to valid range
        open_percent = max(0, min(100, float(open_percent)))
        return self.set_arm_joint("gripper", open_percent)

    def open_gripper(self) -> Dict[str, float]:
        """Fully open the gripper."""
        return self.set_gripper(100)

    def close_gripper(self) -> Dict[str, float]:
        """Fully close the gripper."""
        return self.set_gripper(0)

    def reset_arm_position(self) -> Dict[str, float]:
        """Reset arm to a safe neutral position."""
        neutral = {
            "shoulder_pan": 0,
            "shoulder_lift": 0,
            "elbow_flex": 0,
            "wrist_flex": 0,
            "wrist_roll": 0,
            "gripper": 50,  # Half open
        }
        return self.set_arm_position(neutral)

    def disconnect(self) -> None:
        self._wheels_stop()
        self.wheel_bus.disconnect()
        self.head_bus.disconnect()

    def __del__(self) -> None:
        if hasattr(self, "wheel_bus") and self.wheel_bus.is_connected:
            self.disconnect()

