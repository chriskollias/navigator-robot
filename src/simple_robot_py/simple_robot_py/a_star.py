"""
A* Pathfinding Algorithm
"""

import heapq
import math


class AStar:
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = set(obstacles)  # Convert to set for faster lookups
    
    def heuristic(self, a, b):
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def get_neighbors(self, node):
        """Get valid neighboring cells"""
        x, y = node
        neighbors = []
        
        # Check 8 directions (including diagonals)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue  # Skip the current cell
                
                nx, ny = x + dx, y + dy
                
                # Check if neighbor is within bounds and not an obstacle
                if (0 <= nx < self.width and 0 <= ny < self.height and 
                        (nx, ny) not in self.obstacles):
                    # Additional check for moving diagonally through obstacles
                    if dx != 0 and dy != 0:
                        if (x + dx, y) in self.obstacles or (x, y + dy) in self.obstacles:
                            continue  # Can't squeeze through diagonal gap
                    
                    neighbors.append((nx, ny))
        
        return neighbors
    
    def find_path(self, start, goal):
        """Find path using A* algorithm"""
        # Convert to integer grid coordinates
        start = (int(round(start[0])), int(round(start[1])))
        goal = (int(round(goal[0])), int(round(goal[1])))
        
        # Check if start or goal is invalid
        if start in self.obstacles or goal in self.obstacles:
            return None
        
        # The open set (priority queue)
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        # For node n, came_from[n] is the node immediately preceding it on the path
        came_from = {}
        
        # For node n, g_score[n] is the cost of the cheapest path from start to n
        g_score = {start: 0}
        
        # For node n, f_score[n] = g_score[n] + heuristic(n, goal)
        f_score = {start: self.heuristic(start, goal)}
        
        # Nodes we've seen but not fully processed yet
        open_set_hash = {start}
        
        while open_set:
            # Get node with lowest f_score
            current_f, current = heapq.heappop(open_set)
            open_set_hash.remove(current)
            
            # If we reached the goal, reconstruct and return the path
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            # Check all neighbors
            for neighbor in self.get_neighbors(current):
                # Calculate tentative g_score
                # 1.4 for diagonal, 1.0 for cardinal directions
                dx, dy = neighbor[0] - current[0], neighbor[1] - current[1]
                move_cost = 1.4 if dx != 0 and dy != 0 else 1.0
                tentative_g_score = g_score[current] + move_cost
                
                # If we found a better path to this neighbor
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # Record this better path
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        open_set_hash.add(neighbor)
        
        # No path found
        return None
