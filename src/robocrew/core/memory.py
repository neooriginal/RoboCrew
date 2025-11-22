"""Memory system for robots using SQLite database."""
import sqlite3
import json
from datetime import datetime
from typing import Optional, List, Dict, Any
from pathlib import Path


class MemoryStore:
    """SQLite-based memory store for robot spatial and contextual information."""
    
    def __init__(self, db_path: str = "robot_memory.db"):
        """
        Initialize the memory store.
        
        Args:
            db_path: Path to the SQLite database file
        """
        self.db_path = db_path
        self.conn = sqlite3.connect(db_path)
        self.conn.row_factory = sqlite3.Row
        self._create_tables()
    
    def _create_tables(self):
        """Create necessary database tables if they don't exist."""
        cursor = self.conn.cursor()
        
        # Create memories table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS memories (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                memory_type TEXT NOT NULL,
                content TEXT NOT NULL,
                metadata TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        # Create index for faster searches
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_memory_type 
            ON memories(memory_type)
        """)
        
        self.conn.commit()
    
    def store(self, memory_type: str, content: str, metadata: Optional[Dict[str, Any]] = None) -> int:
        """
        Store a memory in the database.
        
        Args:
            memory_type: Type of memory (e.g., 'spatial', 'object', 'navigation')
            content: The actual memory content/description
            metadata: Optional additional metadata as dictionary
            
        Returns:
            The ID of the stored memory
        """
        cursor = self.conn.cursor()
        metadata_json = json.dumps(metadata) if metadata else None
        
        cursor.execute("""
            INSERT INTO memories (memory_type, content, metadata)
            VALUES (?, ?, ?)
        """, (memory_type, content, metadata_json))
        
        self.conn.commit()
        return cursor.lastrowid
    
    def retrieve_all(self, memory_type: Optional[str] = None, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Retrieve memories from the database.
        
        Args:
            memory_type: Optional filter by memory type
            limit: Optional limit on number of results
            
        Returns:
            List of memory dictionaries
        """
        cursor = self.conn.cursor()
        
        if memory_type:
            query = """
                SELECT id, memory_type, content, metadata, created_at
                FROM memories
                WHERE memory_type = ?
                ORDER BY created_at DESC
            """
            params = [memory_type]
        else:
            query = """
                SELECT id, memory_type, content, metadata, created_at
                FROM memories
                ORDER BY created_at DESC
            """
            params = []
        
        if limit:
            query += " LIMIT ?"
            params.append(limit)
        
        cursor.execute(query, params)
        rows = cursor.fetchall()
        
        memories = []
        for row in rows:
            memory = {
                'id': row['id'],
                'memory_type': row['memory_type'],
                'content': row['content'],
                'metadata': json.loads(row['metadata']) if row['metadata'] else None,
                'created_at': row['created_at']
            }
            memories.append(memory)
        
        return memories
    
    def search(self, keyword: str, memory_type: Optional[str] = None, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Search for memories containing a keyword.
        
        Args:
            keyword: Keyword to search for in content
            memory_type: Optional filter by memory type
            limit: Optional limit on number of results
            
        Returns:
            List of matching memory dictionaries
        """
        cursor = self.conn.cursor()
        
        if memory_type:
            query = """
                SELECT id, memory_type, content, metadata, created_at
                FROM memories
                WHERE memory_type = ? AND content LIKE ?
                ORDER BY created_at DESC
            """
            params = [memory_type, f"%{keyword}%"]
        else:
            query = """
                SELECT id, memory_type, content, metadata, created_at
                FROM memories
                WHERE content LIKE ?
                ORDER BY created_at DESC
            """
            params = [f"%{keyword}%"]
        
        if limit:
            query += " LIMIT ?"
            params.append(limit)
        
        cursor.execute(query, params)
        rows = cursor.fetchall()
        
        memories = []
        for row in rows:
            memory = {
                'id': row['id'],
                'memory_type': row['memory_type'],
                'content': row['content'],
                'metadata': json.loads(row['metadata']) if row['metadata'] else None,
                'created_at': row['created_at']
            }
            memories.append(memory)
        
        return memories
    
    def clear_all(self):
        """Clear all memories from the database. Use with caution!"""
        cursor = self.conn.cursor()
        cursor.execute("DELETE FROM memories")
        self.conn.commit()
    
    def close(self):
        """Close the database connection."""
        if self.conn:
            self.conn.close()
    
    def __del__(self):
        """Cleanup on deletion."""
        self.close()
