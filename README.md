# 🤖 RoboCrew

**Build AI-powered robots that see, move, and manipulate objects — in a few lines of code.**

RoboCrew makes it stupidly simple to create LLM agents for physical robots. Think of it like building agents with CrewAI or AutoGen, except your agents live in the real world with cameras, microphones, wheels, and arms.

![xlerobot_schema](images/main-coming.png)

## Features

- 👁️ **Vision** - Camera feed with automatic angle grid overlay for spatial understanding
- 🎤 **Voice** - Wake-word activated voice commands
- 🧠 **Intelligence** - LLM agent robot control provides complete autonomy and decision making
- 🚗 **Movement** - Pre-built wheel controls for mobile robots
- 🧠 **Memory** - Built-in SQLite-based memory system to remember locations and spatial relationships
- 🦾 **Manipulation** *(coming soon)* - VLA models as a tools for arms control
- 🗺️ **Navigation** *(coming soon)* - Navigation features

## Supported Robots

- **XLeRobot**
- **LeKiwi** (not tested, but we assume XLeRobot code will work for it as platforms are similar. Please try to run repo on your LeKiwi and open an issue with info if it works)
- More robot platforms coming soon!


## Quick Start

```bash
pip install robocrew
```

### Mobile Robot (XLeRobot)

```python
from robocrew.core.LLMAgent import LLMAgent
from robocrew.core.tools import finish_task
from robocrew.robots.XLeRobot.tools import create_move_forward, create_turn_left, create_turn_right
from robocrew.robots.XLeRobot.wheel_controls import XLeRobotWheels

# Set up wheels
sdk = XLeRobotWheels.connect_serial("/dev/ttyUSB0")     # provide the right arm usb port - the arm connected to wheels
wheel_controller = XLeRobotWheels(sdk)

# Create movement tools
move_forward = create_move_forward(wheel_controller)
turn_left = create_turn_left(wheel_controller)
turn_right = create_turn_right(wheel_controller)

# Create agent
agent = LLMAgent(
    model="google_genai:gemini-robotics-er-1.5-preview",
    tools=[move_forward, turn_left, turn_right, finish_task],
    main_camera_usb_port="/dev/video0",  # provide usb port main camera connected to
)
agent.task = "Find kitchen in my house and go there."

agent.go()  # Robot explores autonomously
```

### With Voice Commands

Add a microphone to give your robot voice-activated tasks:

```python
agent = LLMAgent(
    model="google_genai:gemini-robotics-er-1.5-preview",
    tools=[move_forward, turn_left, turn_right],
    main_camera_usb_port="/dev/video0",  # provide usb port main camera connected to
    sounddevice_index=0,  # Your mic device
    wakeword="robot",  # The robot listens for this word in your speech
    history_len=4,
)
```

Then install Portaudio for audio support:
```bash
sudo apt install portaudio19-dev
```

Now just say something like **"Hey robot, bring me a beer."** — the robot listens continuously and when it hears the word "robot" anywhere in your command, it'll use the entire phrase as its new task.

### With Memory System

Enable the memory system so your robot can remember locations and complete tasks faster:

```python
agent = LLMAgent(
    model="google_genai:gemini-robotics-er-1.5-preview",
    tools=[move_forward, turn_left, turn_right, finish_task],
    main_camera_usb_port="/dev/video0",
    memory_enabled=True,  # Enable memory system
    memory_db_path="robot_memory.db",  # SQLite database path
)
```

The robot automatically gets two new tools:
- **store_memory**: Save spatial information like "Bathroom is next to kitchen"
- **retrieve_memory**: Search and recall previously stored information

Benefits:
- **Faster navigation**: Robot remembers room locations instead of exploring from scratch
- **Persistent knowledge**: Memories survive between sessions
- **Smart exploration**: Robot builds a mental map as it learns

Example memories the robot can store:
```python
# Spatial relationships
"Kitchen is next to bathroom"
"Living room has large windows"

# Object locations  
"Backpack is usually in the bedroom"
"TV remote on living room table"

# Navigation hints
"Turn left at red door to reach kitchen"
"Stairs are at the end of the hallway"
```


## Key Parameters

- **model**: Any LangChain model
- **tools**: List of functions your robot can call (movement, manipulation)
- **main_camera_usb_port**: Your camera device (find with `ls /dev/video*`)
- **sounddevice_index**: Microphone index (optional, for voice commands)
- **wakeword**: Word that must appear in your speech to give robot a new task (default: "robot").
- **history_len**: How many conversation turns to remember (optional)
- **memory_enabled**: Enable the memory system (default: False)
- **memory_db_path**: Path to SQLite database for memory storage (default: "robot_memory.db")


## Custom Tools

Create your own tools easily:

```python
from langchain_core.tools import tool

@tool
def grab_object(name: str) -> str:
    """Grab the specified object."""
    # Your hardware code here
    return f"Grabbed {name}"

# Then just add to tools list
agent = LLMAgent(tools=[grab_object, finish_task], ...)
```


