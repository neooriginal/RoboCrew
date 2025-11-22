"""Example demonstrating object manipulation with XLeRobot arms."""

from robocrew.core.LLMAgent import LLMAgent
from robocrew.core.tools import finish_task
from robocrew.robots.XLeRobot.wheel_controls import XLeRobotWheels
from robocrew.robots.XLeRobot.arm_controls import XLeRobotArms
from robocrew.robots.XLeRobot.tools import (
    create_move_forward, 
    create_turn_left, 
    create_turn_right
)
from robocrew.robots.XLeRobot.manipulation_tools import (
    create_move_arm_to,
    create_open_gripper,
    create_close_gripper,
    create_reset_arm,
    create_reach_object,
    create_grasp_object,
    create_release_object,
)

# System prompt for manipulation-capable robot
system_prompt = """You are a mobile household robot with two arms and grippers.

You can:
- Navigate with wheels (move forward/backward, turn left/right)
- Control both arms independently (left and right)
- Open and close grippers to pick up and release objects
- See your environment through a camera with angle markers

When manipulating objects:
- Use the camera view to locate objects and note their angle
- Use reach_object or grasp_object with the appropriate angle
- The right arm is typically used for primary manipulation
- Always reset arms to home position when done with a task
"""

# Initialize hardware controllers
print("Initializing robot hardware...")

# Wheel controller for navigation
# Note: XLeRobot uses one arm's motors for wheels (typically right arm base motors)
wheel_controller = XLeRobotWheels(
    wheel_arm_usb="/dev/arm_right",  # USB port for wheel control (base motors)
    head_arm_usb="/dev/arm_head"
)

# Arm controller for manipulation
# Note: If your robot uses the same USB for wheels and arm, only initialize one arm
arm_controller = XLeRobotArms(
    left_arm_usb="/dev/arm_left",
    right_arm_usb="/dev/ttyUSB1",  # Use different USB port from wheels
    enable_left=True,   # Enable both arms
    enable_right=True
)

# Create movement tools
move_forward = create_move_forward(wheel_controller)
turn_left = create_turn_left(wheel_controller)
turn_right = create_turn_right(wheel_controller)

# Create manipulation tools
move_arm_to = create_move_arm_to(arm_controller)
open_gripper = create_open_gripper(arm_controller)
close_gripper = create_close_gripper(arm_controller)
reset_arm = create_reset_arm(arm_controller)
reach_object = create_reach_object(arm_controller)
grasp_object = create_grasp_object(arm_controller)
release_object = create_release_object(arm_controller)

# Initialize LLM agent with all capabilities
agent = LLMAgent(
    model="google_genai:gemini-robotics-er-1.5-preview",
    system_prompt=system_prompt,
    tools=[
        # Movement tools
        move_forward,
        turn_left,
        turn_right,
        # Manipulation tools
        move_arm_to,
        open_gripper,
        close_gripper,
        reset_arm,
        reach_object,
        grasp_object,
        release_object,
        # Control
        finish_task,
    ],
    main_camera_usb_port="/dev/video0",  # Adjust to your camera port
    camera_fov=120,
    history_len=4,
    debug_mode=False,
)

print("Robot initialized successfully!")
print("Starting manipulation task...\n")

# Example task: Pick up an object
agent.task = "Look around for a cup or bottle. When you find it, reach toward it with your right arm and grasp it. Then reset your arm to home position."

try:
    result = agent.go()
    print(f"\nTask completed: {result}")
except KeyboardInterrupt:
    print("\nTask interrupted by user.")
except Exception as e:
    print(f"\nTask failed with error: {e}")
finally:
    # Clean up
    print("Cleaning up...")
    arm_controller.reset_both_arms()
    arm_controller.disconnect()
    wheel_controller.disconnect()
    print("Done!")
