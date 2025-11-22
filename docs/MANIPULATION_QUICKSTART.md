# Object Manipulation Quick Start Guide

This guide helps you get started with RoboCrew's object manipulation features.

## Overview

RoboCrew now supports robotic arm control for object manipulation, enabling your robot to:
- Reach toward objects
- Grasp and release objects
- Control individual joints
- Open and close grippers
- Reset arms to safe positions

## Basic Setup

### 1. Initialize Arm Controller

```python
from robocrew.robots.XLeRobot.arm_controls import XLeRobotArms

# Initialize both arms
arm_controller = XLeRobotArms(
    left_arm_usb="/dev/arm_left",
    right_arm_usb="/dev/ttyUSB1",
    enable_left=True,
    enable_right=True
)

# Or just one arm
arm_controller = XLeRobotArms(
    left_arm_usb="/dev/arm_left",
    enable_left=True,
    enable_right=False  # Disable right arm
)
```

### 2. Create Manipulation Tools

```python
from robocrew.robots.XLeRobot.manipulation_tools import (
    create_reach_object,
    create_grasp_object,
    create_release_object,
    create_open_gripper,
    create_close_gripper,
    create_reset_arm,
)

# Create tools
reach_object = create_reach_object(arm_controller)
grasp_object = create_grasp_object(arm_controller)
release_object = create_release_object(arm_controller)
open_gripper = create_open_gripper(arm_controller)
close_gripper = create_close_gripper(arm_controller)
reset_arm = create_reset_arm(arm_controller)
```

### 3. Add to LLM Agent

```python
from robocrew.core.LLMAgent import LLMAgent
from robocrew.core.tools import finish_task

agent = LLMAgent(
    model="google_genai:gemini-robotics-er-1.5-preview",
    tools=[
        reach_object,
        grasp_object,
        release_object,
        open_gripper,
        close_gripper,
        reset_arm,
        finish_task,
    ],
    main_camera_usb_port="/dev/video0",
)

agent.task = "Pick up the cup on the table"
agent.go()
```

## Available Tools

### High-Level Tools (Recommended)

**`reach_object(arm="right", angle_degrees=0.0, extend=True)`**
- Reach toward an object at a specific angle
- Use with camera view to determine angle
- Example: `reach_object(arm="right", angle_degrees=15.0)`

**`grasp_object(arm="right", angle_degrees=0.0)`**
- Reach toward object and close gripper
- Complete grasping action in one step
- Example: `grasp_object(arm="right", angle_degrees=-10.0)`

**`release_object(arm="right")`**
- Open gripper to release grasped object
- Example: `release_object(arm="left")`

**`reset_arm(arm="both")`**
- Move arm(s) to safe home position
- Options: "left", "right", or "both"
- Example: `reset_arm(arm="both")`

### Low-Level Tools (Advanced)

**`open_gripper(arm="right")`**
- Fully open the gripper
- Example: `open_gripper(arm="left")`

**`close_gripper(arm="right")`**
- Fully close the gripper
- Example: `close_gripper(arm="right")`

**`set_gripper(arm="right", opening_percentage=50.0)`**
- Set gripper to specific opening (0-100%)
- Example: `set_gripper(arm="right", opening_percentage=75.0)`

**`move_arm_to(arm, shoulder_pan=None, shoulder_lift=None, elbow=None, wrist_rotate=None, wrist_tilt=None)`**
- Move specific joints to target positions
- All angles in degrees
- Example: `move_arm_to(arm="right", shoulder_pan=30, elbow=100)`

## Joint Ranges and Limits

All joints have safety limits enforced:

| Joint | Range (degrees) | Default Home |
|-------|----------------|--------------|
| shoulder_pan | -170 to 170 | 0 |
| shoulder_lift | -90 to 90 | 0 |
| elbow | 0 to 150 | 90 |
| wrist_rotate | -180 to 180 | 0 |
| wrist_tilt | -90 to 90 | 0 |
| gripper | 0 to 100 (%) | 50 |

## Example Tasks

### Pick Up Object

```python
agent.task = """
Look for a cup or bottle in your view.
When you find it, note the angle from the camera overlay.
Use grasp_object with that angle to pick it up.
Then reset your arm to home position.
"""
```

### Transfer Object Between Arms

```python
agent.task = """
With your right arm, grasp the object at angle 10 degrees.
Then open your left gripper.
Move to a position where both arms can meet.
Open right gripper to release into left hand.
Reset both arms.
"""
```

### Reach and Inspect

```python
agent.task = """
Reach toward the object at -15 degrees with your right arm.
Do not grasp it, just reach close to examine it.
Report what you observe.
Reset arm when done.
"""
```

## Camera Integration

The robot's camera provides angle markers to help locate objects:

1. Objects appear with degree markers in the camera view
2. Use these angles with `reach_object` or `grasp_object`
3. Positive angles are to the right, negative to the left
4. 0 degrees is straight ahead

Example:
```
Camera shows cup at +20° marker
→ Use: grasp_object(arm="right", angle_degrees=20.0)
```

## Safety Features

### Built-in Safety

- **Joint Limits**: All movements are checked against joint limits
- **Position Validation**: Invalid positions are rejected with error message
- **Emergency Stop**: Call `arm_controller.emergency_stop()` to halt all motion
- **Home Position**: Use `reset_arm()` to return to safe default position

### Best Practices

1. **Always reset arms** after completing a task
2. **Test with reach_object** before grasping
3. **Use try-catch** in custom code for error handling
4. **Monitor robot** during first uses of new tasks
5. **Keep workspace clear** of obstacles

## Troubleshooting

### "Failed to move arm" errors

- Check USB port connections
- Verify motor IDs match your hardware configuration
- Ensure robot is powered on
- Check joint limits aren't exceeded

### Gripper not working

- Verify gripper motor is configured correctly
- Check gripper motor ID (default: 6 for left, 15 for right)
- Test with `open_gripper()` and `close_gripper()` directly

### USB port conflicts

- Wheels and arms need separate USB connections
- If sharing USB, initialize only needed controllers
- Check `/dev/` ports with `ls /dev/tty*`

### LLM not using manipulation tools

- Ensure tools are added to agent's tool list
- Include object location in task description
- Mention using camera angles for guidance
- Try more explicit instructions

## Hardware Configuration

### Default Motor IDs

**Left Arm:**
- Motor 1: shoulder_pan
- Motor 2: shoulder_lift
- Motor 3: elbow
- Motor 4: wrist_rotate
- Motor 5: wrist_tilt
- Motor 6: gripper

**Right Arm:**
- Motor 10: shoulder_pan
- Motor 11: shoulder_lift
- Motor 12: elbow
- Motor 13: wrist_rotate
- Motor 14: wrist_tilt
- Motor 15: gripper

### Custom Configuration

To modify motor IDs or limits, edit:
`src/robocrew/robots/XLeRobot/arm_config.json`

Or update the constants in:
`src/robocrew/robots/XLeRobot/arm_controls.py`

## Next Steps

### Current Capabilities (Phase 1 & 2)
- ✅ Basic arm control
- ✅ Gripper control
- ✅ Simple reaching and grasping
- ✅ LLM agent integration

### Future Enhancements (Phase 3+)
- Vision-Language-Action (VLA) model integration
- Inverse kinematics for precise positioning
- Trajectory planning and collision avoidance
- Force control and tactile feedback
- Bimanual coordination
- Learning from demonstration

See `docs/OBJECT_MANIPULATION_INTEGRATION_PLAN.md` for the full roadmap.

## Complete Example

See `examples/manipulation_demo.py` for a working example that combines:
- Wheel control for navigation
- Arm control for manipulation
- Camera vision for object detection
- LLM agent for autonomous task completion

## Support

For issues or questions:
- Check the troubleshooting section above
- Review `docs/OBJECT_MANIPULATION_INTEGRATION_PLAN.md`
- Open an issue on GitHub
- Check example scripts in `examples/`

Happy manipulating! 🦾
