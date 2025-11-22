"""Manipulation tools for XLeRobot arms."""

import time
from langchain_core.tools import tool
from typing import Optional


def create_move_arm_to(arm_controller):
    """Create a tool for moving arm joints to specific positions."""
    @tool
    def move_arm_to(
        arm: str,
        shoulder_pan: Optional[float] = None,
        shoulder_lift: Optional[float] = None,
        elbow: Optional[float] = None,
        wrist_rotate: Optional[float] = None,
        wrist_tilt: Optional[float] = None,
    ) -> str:
        """
        Move arm joints to specified positions.
        
        Args:
            arm: Which arm to move ('left' or 'right')
            shoulder_pan: Shoulder pan angle in degrees (-170 to 170)
            shoulder_lift: Shoulder lift angle in degrees (-90 to 90)
            elbow: Elbow angle in degrees (0 to 150)
            wrist_rotate: Wrist rotation angle in degrees (-180 to 180)
            wrist_tilt: Wrist tilt angle in degrees (-90 to 90)
        
        Returns:
            Status message indicating success or failure
        """
        try:
            positions = {}
            if shoulder_pan is not None:
                positions["shoulder_pan"] = float(shoulder_pan)
            if shoulder_lift is not None:
                positions["shoulder_lift"] = float(shoulder_lift)
            if elbow is not None:
                positions["elbow"] = float(elbow)
            if wrist_rotate is not None:
                positions["wrist_rotate"] = float(wrist_rotate)
            if wrist_tilt is not None:
                positions["wrist_tilt"] = float(wrist_tilt)
            
            if not positions:
                return "No joint positions specified."
            
            arm_controller.set_joint_positions(arm, positions, validate=True)
            time.sleep(1)  # Wait for movement
            
            joint_list = ", ".join(f"{k}={v}°" for k, v in positions.items())
            return f"Moved {arm} arm: {joint_list}"
            
        except Exception as e:
            return f"Failed to move arm: {str(e)}"
    
    return move_arm_to


def create_open_gripper(arm_controller):
    """Create a tool for opening the gripper."""
    @tool
    def open_gripper(arm: str = "right") -> str:
        """
        Open the gripper on the specified arm.
        
        Args:
            arm: Which arm's gripper to open ('left' or 'right')
        
        Returns:
            Status message
        """
        try:
            arm_controller.open_gripper(arm)
            time.sleep(0.5)
            return f"Opened {arm} gripper."
        except Exception as e:
            return f"Failed to open gripper: {str(e)}"
    
    return open_gripper


def create_close_gripper(arm_controller):
    """Create a tool for closing the gripper."""
    @tool
    def close_gripper(arm: str = "right") -> str:
        """
        Close the gripper on the specified arm to grasp an object.
        
        Args:
            arm: Which arm's gripper to close ('left' or 'right')
        
        Returns:
            Status message
        """
        try:
            arm_controller.close_gripper(arm)
            time.sleep(0.5)
            return f"Closed {arm} gripper."
        except Exception as e:
            return f"Failed to close gripper: {str(e)}"
    
    return close_gripper


def create_set_gripper(arm_controller):
    """Create a tool for setting gripper to specific opening."""
    @tool
    def set_gripper(arm: str, opening_percentage: float) -> str:
        """
        Set gripper opening to a specific percentage.
        
        Args:
            arm: Which arm's gripper to control ('left' or 'right')
            opening_percentage: How open the gripper should be (0=closed, 100=fully open)
        
        Returns:
            Status message
        """
        try:
            arm_controller.set_gripper(arm, opening_percentage)
            time.sleep(0.5)
            return f"Set {arm} gripper to {opening_percentage}% open."
        except Exception as e:
            return f"Failed to set gripper: {str(e)}"
    
    return set_gripper


def create_reset_arm(arm_controller):
    """Create a tool for resetting arm to home position."""
    @tool
    def reset_arm(arm: str = "both") -> str:
        """
        Move arm(s) to safe home position.
        
        Args:
            arm: Which arm to reset ('left', 'right', or 'both')
        
        Returns:
            Status message
        """
        try:
            if arm == "both":
                arm_controller.reset_both_arms()
                return "Reset both arms to home position."
            else:
                arm_controller.reset_arm(arm)
                return f"Reset {arm} arm to home position."
        except Exception as e:
            return f"Failed to reset arm: {str(e)}"
    
    return reset_arm


def create_reach_object(arm_controller):
    """Create a tool for reaching toward an object (simplified version)."""
    @tool
    def reach_object(
        arm: str,
        angle_degrees: float,
        extend: bool = True
    ) -> str:
        """
        Reach toward an object at a specified horizontal angle.
        
        This is a simplified reaching motion that extends the arm forward at the given angle.
        
        Args:
            arm: Which arm to use ('left' or 'right')
            angle_degrees: Horizontal angle where object is located (-60 to 60 degrees, 0 is straight ahead)
            extend: Whether to extend the arm forward (True) or keep it neutral (False)
        
        Returns:
            Status message
        """
        try:
            # Clamp angle to reasonable range
            angle_degrees = max(-60, min(60, angle_degrees))
            
            # Simple reaching motion: pan toward object and extend arm
            positions = {
                "shoulder_pan": float(angle_degrees),
                "shoulder_lift": -20 if extend else 0,  # Lower shoulder to reach forward
                "elbow": 110 if extend else 90,  # Extend elbow
            }
            
            arm_controller.set_joint_positions(arm, positions, validate=True)
            time.sleep(1.5)
            
            return f"{arm.capitalize()} arm reaching toward object at {angle_degrees}° {'(extended)' if extend else '(neutral)'}."
        except Exception as e:
            return f"Failed to reach: {str(e)}"
    
    return reach_object


def create_grasp_object(arm_controller):
    """Create a tool for grasping (reach + close gripper)."""
    @tool
    def grasp_object(arm: str, angle_degrees: float) -> str:
        """
        Reach toward an object and attempt to grasp it.
        
        Args:
            arm: Which arm to use ('left' or 'right')
            angle_degrees: Horizontal angle where object is located (-60 to 60 degrees)
        
        Returns:
            Status message
        """
        try:
            # Reach toward object
            angle_degrees = max(-60, min(60, angle_degrees))
            positions = {
                "shoulder_pan": float(angle_degrees),
                "shoulder_lift": -25,
                "elbow": 115,
            }
            arm_controller.set_joint_positions(arm, positions, validate=True)
            time.sleep(1.5)
            
            # Close gripper to grasp
            arm_controller.close_gripper(arm)
            time.sleep(0.8)
            
            return f"{arm.capitalize()} arm grasped object at {angle_degrees}°."
        except Exception as e:
            return f"Failed to grasp object: {str(e)}"
    
    return grasp_object


def create_release_object(arm_controller):
    """Create a tool for releasing a grasped object."""
    @tool
    def release_object(arm: str) -> str:
        """
        Release the currently grasped object by opening the gripper.
        
        Args:
            arm: Which arm to use ('left' or 'right')
        
        Returns:
            Status message
        """
        try:
            arm_controller.open_gripper(arm)
            time.sleep(0.5)
            return f"{arm.capitalize()} arm released object."
        except Exception as e:
            return f"Failed to release object: {str(e)}"
    
    return release_object
