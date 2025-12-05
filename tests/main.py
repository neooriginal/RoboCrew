import cv2
from robocrew.core.tools import finish_task
from robocrew.core.LLMAgent import LLMAgent
from robocrew.robots.XLeRobot.tools import (
    create_move_forward,
    create_turn_left,
    create_turn_right,
    create_look_around,
    create_move_backward,
)
from robocrew.robots.XLeRobot.servo_controls import ServoControler
from web_server import RobotWebServer


# Custom system prompt
prompt = "You are a mobile household robot with two arms."

# Set up main camera (mounted on head)
main_camera_usb_port = "/dev/video0"  # camera usb port Eg: /dev/video0
main_camera = cv2.VideoCapture(main_camera_usb_port)
main_camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Set up servo controller for wheels and head
wheel_arm_usb = "/dev/robot_acm0"    # provide your right arm usb port. Eg: /de>
head_arm_usb = "/dev/robot_acm1"      # provide your left arm usb port. Eg: /de>
wheel_controller = ServoControler(wheel_arm_usb, head_arm_usb)

# Create movement and head control tools
move_forward = create_move_forward(wheel_controller)
turn_left = create_turn_left(wheel_controller)
turn_right = create_turn_right(wheel_controller)
look_around = create_look_around(wheel_controller, main_camera)
move_backward = create_move_backward(wheel_controller)

# Initialize agent
agent = LLMAgent(
    model="google_genai:gemini-robotics-er-1.5-preview",
    system_prompt=prompt,
    tools=[
        move_forward,
        turn_left,
        turn_right,
        look_around,
        move_backward,
        finish_task,
    ],
    history_len=6,  # nr of last message-answer pairs to keep
    main_camera_usb_port=main_camera,  # provide main camera object
    camera_fov=85,
    sounddevice_index=None,  # set to your microphone sounddevice index if you >
    use_memory=True,  # enable long-term memory
    debug_mode=False,
)

print("Agent initialized.")

# Start web server in background thread
web_server = RobotWebServer(wheel_controller, main_camera)
web_server.run_in_thread()
print("Web server started at http://localhost:5000")
print("Open your browser to control the robot with WASD and mouse!")

# Run agent with a sample task
agent.task = "Explore and map the house using memory. Never run against walls."
agent.go()

# Clean up
wheel_controller.disconnect()
main_camera.release()

