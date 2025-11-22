"""Simple demonstration of the memory system without requiring robot hardware.

This script shows how the memory system works independently of the robot.
"""
from robocrew.core.memory import MemoryStore

print("=" * 60)
print("RoboCrew Memory System Demo")
print("=" * 60)

# Create a memory store (uses robot_demo_memory.db)
with MemoryStore("robot_demo_memory.db") as memory:
    print("\n✓ Memory store initialized")
    
    # Simulate robot learning about its environment
    print("\n🤖 Robot is exploring and learning...")
    
    # Store spatial information
    memory.store("spatial", "Kitchen is next to the bathroom")
    memory.store("spatial", "Living room has large windows facing south")
    memory.store("spatial", "Bedroom is at the end of the hallway")
    print("   ✓ Stored 3 spatial memories")
    
    # Store object locations
    memory.store("object", "TV remote is usually on the living room table")
    memory.store("object", "Keys are in the bowl by the front door")
    print("   ✓ Stored 2 object location memories")
    
    # Store navigation hints
    memory.store("navigation", "Turn left at the red door to reach kitchen")
    memory.store("navigation", "Stairs are at the end of the main hallway")
    print("   ✓ Stored 2 navigation memories")
    
    # Now simulate using memories to complete a task
    print("\n🎯 New task: Find the kitchen")
    print("   Searching memory for 'kitchen'...")
    
    kitchen_memories = memory.search("kitchen")
    print(f"\n   Found {len(kitchen_memories)} relevant memory/memories:")
    for i, mem in enumerate(kitchen_memories, 1):
        print(f"   {i}. [{mem['memory_type']}] {mem['content']}")
    
    print("\n   ✅ Using memory, robot knows:")
    print("      - Kitchen is next to the bathroom")
    print("      - Can turn left at red door to reach it")
    print("      - No need to explore from scratch!")
    
    # Show all spatial memories
    print("\n🗺️  All spatial memories:")
    spatial = memory.retrieve_all(memory_type="spatial")
    for mem in spatial:
        print(f"   • {mem['content']}")
    
    # Show memory stats
    all_memories = memory.retrieve_all()
    print(f"\n📊 Total memories stored: {len(all_memories)}")
    print("   These will persist for future tasks!")

print("\n✓ Memory store closed\n")
print("=" * 60)
print("Demo complete! Memory saved to: robot_demo_memory.db")
print("Run this script again to see memories persist between sessions.")
print("=" * 60)
