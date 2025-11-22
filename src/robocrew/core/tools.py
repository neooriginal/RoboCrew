from langchain_core.tools import tool


@tool
def finish_task():
    """Claim that task is finished and go idle. You need to ensure the task is actually finished before calling this tool."""
    return "Task finished"


def create_store_memory(memory_store):
    """Create a tool for storing memories in the robot's memory system."""
    @tool
    def store_memory(memory_type: str, content: str) -> str:
        """
        Store a memory for future reference. This helps the robot remember locations, objects, and spatial relationships.
        
        Args:
            memory_type: Type of memory - use 'spatial' for room locations and relationships (e.g., "kitchen next to bathroom"), 
                        'object' for object locations (e.g., "backpack in bedroom"), 
                        or 'navigation' for navigation hints (e.g., "turn left at red door to reach kitchen")
            content: Description of what to remember (e.g., "Bathroom is next to kitchen", "Red door leads to kitchen")
        
        Returns:
            Confirmation message with the memory ID
        """
        memory_id = memory_store.store(memory_type, content)
        return f"Memory stored successfully with ID {memory_id}. Type: {memory_type}, Content: {content}"
    
    return store_memory


def create_retrieve_memory(memory_store):
    """Create a tool for retrieving memories from the robot's memory system."""
    @tool
    def retrieve_memory(keyword: str, memory_type: str = None) -> str:
        """
        Search and retrieve stored memories. Use this to recall previously learned information about locations, objects, or navigation.
        
        Args:
            keyword: What to search for (e.g., "kitchen", "bathroom", "backpack")
            memory_type: Optional filter by type - 'spatial', 'object', or 'navigation'. Leave empty to search all types.
        
        Returns:
            A formatted string with matching memories, or a message if no memories found
        """
        memories = memory_store.search(keyword, memory_type, limit=10)
        
        if not memories:
            return f"No memories found for keyword '{keyword}'" + (f" in type '{memory_type}'" if memory_type else "")
        
        result = f"Found {len(memories)} relevant memory/memories:\n"
        for i, mem in enumerate(memories, 1):
            result += f"{i}. [{mem['memory_type']}] {mem['content']} (stored: {mem['created_at']})\n"
        
        return result
    
    return retrieve_memory

