"""Example of using RoboCrew with memory system enabled.

This example shows how to create a robot that can remember locations,
objects, and spatial relationships to complete tasks more efficiently.
"""
from robocrew.core.tools import finish_task
from robocrew.core.LLMAgent import LLMAgent
from robocrew.robots.XLeRobot.tools import create_move_forward, create_turn_left, create_turn_right
from robocrew.robots.XLeRobot.wheel_controls import XLeRobotWheels

# System prompt with memory instructions
prompt = """You are a mobile household robot with two arms and a memory system.

MEMORY USAGE:
- Use store_memory to save important spatial information (e.g., "Kitchen is next to bathroom")
- Use retrieve_memory to recall previously learned locations before exploring
- Memory types: 'spatial' (room relationships), 'object' (object locations), 'navigation' (navigation hints)

STRATEGY:
1. When given a task to find a location, first check your memory using retrieve_memory
2. If you have relevant memories, use them to navigate efficiently
3. As you explore, store new spatial information for future reference
4. This helps you complete repeated tasks much faster!

Example memories:
- store_memory(memory_type="spatial", content="Bathroom is next to kitchen")
- store_memory(memory_type="object", content="TV remote usually on living room table")
- store_memory(memory_type="navigation", content="Turn left at red door to reach kitchen")
"""

# Set up wheel movement tools
wheel_arm_usb = "/dev/arm_right"    # provide your right arm usb port. Eg: /dev/ttyACM1
wheel_controller = XLeRobotWheels(wheel_arm_usb)
move_forward = create_move_forward(wheel_controller)
turn_left = create_turn_left(wheel_controller)
turn_right = create_turn_right(wheel_controller)

# Initialize agent with memory enabled
agent = LLMAgent(
    model="google_genai:gemini-robotics-er-1.5-preview",
    system_prompt=prompt,
    tools=[
        move_forward,
        turn_left,
        turn_right,
        finish_task,
    ],
    history_len=4,  # nr of last message-answer pairs to keep
    main_camera_usb_port="/dev/camera_center",  # provide usb port main camera. Eg: /dev/video0
    camera_fov=120,
    sounddevice_index=0,  # index of your microphone sounddevice (optional)
    debug_mode=False,
    
    # Memory system parameters
    memory_enabled=True,  # Enable the memory system
    memory_db_path="robot_memory.db",  # Path to memory database (will be created if doesn't exist)
)

print("Agent initialized with memory system.")
print("Memory tools available: store_memory, retrieve_memory")

# Example task that benefits from memory
# First time: robot explores and learns
# Second time: robot uses memory to navigate more efficiently
agent.task = "Find the kitchen and go there. Remember spatial relationships for future tasks."
agent.go()
