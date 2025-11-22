"""Arm control utilities for XLeRobot manipulation."""

from __future__ import annotations

import time
from typing import Dict, Optional, Tuple

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

DEFAULT_BAUDRATE = 1_000_000

# Motor IDs for each arm (example configuration for 6-DOF arms)
LEFT_ARM_MOTOR_IDS = {
    1: "shoulder_pan",
    2: "shoulder_lift", 
    3: "elbow",
    4: "wrist_rotate",
    5: "wrist_tilt",
    6: "gripper",
}

RIGHT_ARM_MOTOR_IDS = {
    10: "shoulder_pan",
    11: "shoulder_lift",
    12: "elbow", 
    13: "wrist_rotate",
    14: "wrist_tilt",
    15: "gripper",
}

# Joint limits (in degrees) for safety
JOINT_LIMITS = {
    "shoulder_pan": (-170, 170),
    "shoulder_lift": (-90, 90),
    "elbow": (0, 150),
    "wrist_rotate": (-180, 180),
    "wrist_tilt": (-90, 90),
    "gripper": (0, 100),  # 0=closed, 100=open
}

# Home positions (safe default positions)
HOME_POSITIONS = {
    "shoulder_pan": 0,
    "shoulder_lift": 0,
    "elbow": 90,
    "wrist_rotate": 0,
    "wrist_tilt": 0,
    "gripper": 50,  # Half open
}


class XLeRobotArms:
    """Controller for XLeRobot arm motors with dual arm support."""

    def __init__(
        self,
        left_arm_usb: str = "/dev/arm_left",
        right_arm_usb: str = "/dev/arm_right",
        *,
        enable_left: bool = True,
        enable_right: bool = True,
    ) -> None:
        """
        Initialize arm controllers.
        
        Args:
            left_arm_usb: USB port for left arm
            right_arm_usb: USB port for right arm
            enable_left: Whether to initialize left arm
            enable_right: Whether to initialize right arm
        """
        self.left_arm_usb = left_arm_usb
        self.right_arm_usb = right_arm_usb
        
        # Initialize left arm if enabled
        self.left_bus = None
        if enable_left:
            left_motors = {
                motor_id: Motor(motor_id, "sts3215", MotorNormMode.DEGREES)
                for motor_id in LEFT_ARM_MOTOR_IDS.keys()
            }
            self.left_bus = FeetechMotorsBus(
                port=left_arm_usb,
                motors=left_motors,
            )
            self.left_bus.connect()
            self._apply_arm_modes(self.left_bus, LEFT_ARM_MOTOR_IDS)
        
        # Initialize right arm if enabled
        self.right_bus = None
        if enable_right:
            right_motors = {
                motor_id: Motor(motor_id, "sts3215", MotorNormMode.DEGREES)
                for motor_id in RIGHT_ARM_MOTOR_IDS.keys()
            }
            self.right_bus = FeetechMotorsBus(
                port=right_arm_usb,
                motors=right_motors,
            )
            self.right_bus.connect()
            self._apply_arm_modes(self.right_bus, RIGHT_ARM_MOTOR_IDS)

    def _apply_arm_modes(self, bus: FeetechMotorsBus, motor_map: Dict) -> None:
        """Apply position control mode to all arm motors."""
        from lerobot.motors.feetech import OperatingMode
        
        for motor_id in motor_map.keys():
            bus.write("Operating_Mode", motor_id, OperatingMode.POSITION.value)
        
        bus.enable_torque()

    def _get_bus(self, arm: str) -> FeetechMotorsBus:
        """Get the motor bus for specified arm."""
        if arm == "left":
            if self.left_bus is None:
                raise ValueError("Left arm not initialized")
            return self.left_bus
        elif arm == "right":
            if self.right_bus is None:
                raise ValueError("Right arm not initialized")
            return self.right_bus
        else:
            raise ValueError(f"Invalid arm: {arm}. Must be 'left' or 'right'")

    def _get_motor_map(self, arm: str) -> Dict:
        """Get motor ID mapping for specified arm."""
        if arm == "left":
            return LEFT_ARM_MOTOR_IDS
        elif arm == "right":
            return RIGHT_ARM_MOTOR_IDS
        else:
            raise ValueError(f"Invalid arm: {arm}")

    def set_joint_positions(
        self, 
        arm: str, 
        positions: Dict[str, float],
        validate: bool = True
    ) -> None:
        """
        Set positions for multiple joints.
        
        Args:
            arm: Which arm ('left' or 'right')
            positions: Dict mapping joint names to target positions (degrees)
            validate: Whether to validate positions against limits
        """
        bus = self._get_bus(arm)
        motor_map = self._get_motor_map(arm)
        
        # Validate positions if requested
        if validate:
            for joint_name, position in positions.items():
                if joint_name not in JOINT_LIMITS:
                    raise ValueError(f"Unknown joint: {joint_name}")
                min_pos, max_pos = JOINT_LIMITS[joint_name]
                if not (min_pos <= position <= max_pos):
                    raise ValueError(
                        f"Position {position} for {joint_name} outside limits "
                        f"[{min_pos}, {max_pos}]"
                    )
        
        # Convert joint names to motor IDs and prepare payload
        payload = {}
        for joint_name, position in positions.items():
            # Find motor ID for this joint
            motor_id = None
            for mid, jname in motor_map.items():
                if jname == joint_name:
                    motor_id = mid
                    break
            if motor_id is not None:
                payload[motor_id] = float(position)
        
        if payload:
            bus.sync_write("Goal_Position", payload)

    def get_joint_positions(self, arm: str) -> Dict[str, float]:
        """
        Get current positions of all joints.
        
        Args:
            arm: Which arm ('left' or 'right')
            
        Returns:
            Dict mapping joint names to current positions (degrees)
        """
        bus = self._get_bus(arm)
        motor_map = self._get_motor_map(arm)
        motor_ids = list(motor_map.keys())
        
        positions = bus.sync_read("Present_Position", motor_ids)
        
        # Convert motor IDs back to joint names
        result = {}
        for motor_id, position in positions.items():
            joint_name = motor_map.get(motor_id)
            if joint_name:
                result[joint_name] = position
        
        return result

    def set_gripper(self, arm: str, open_percentage: float) -> None:
        """
        Set gripper opening.
        
        Args:
            arm: Which arm ('left' or 'right')
            open_percentage: How open the gripper should be (0=closed, 100=open)
        """
        open_percentage = max(0, min(100, open_percentage))
        self.set_joint_positions(arm, {"gripper": open_percentage}, validate=True)

    def open_gripper(self, arm: str) -> None:
        """Fully open the gripper."""
        self.set_gripper(arm, 100)

    def close_gripper(self, arm: str) -> None:
        """Fully close the gripper."""
        self.set_gripper(arm, 0)

    def reset_arm(self, arm: str) -> None:
        """Move arm to safe home position."""
        self.set_joint_positions(arm, HOME_POSITIONS, validate=True)
        time.sleep(2)  # Wait for movement to complete

    def reset_both_arms(self) -> None:
        """Move both arms to safe home positions."""
        if self.left_bus:
            self.reset_arm("left")
        if self.right_bus:
            self.reset_arm("right")

    def emergency_stop(self) -> None:
        """Immediately disable torque on all motors."""
        if self.left_bus:
            self.left_bus.disable_torque()
        if self.right_bus:
            self.right_bus.disable_torque()

    def disconnect(self) -> None:
        """Disconnect from motor buses."""
        if self.left_bus and self.left_bus.is_connected:
            self.left_bus.disconnect()
        if self.right_bus and self.right_bus.is_connected:
            self.right_bus.disconnect()

    def __del__(self) -> None:
        """Cleanup on deletion."""
        if hasattr(self, "left_bus") or hasattr(self, "right_bus"):
            self.disconnect()
