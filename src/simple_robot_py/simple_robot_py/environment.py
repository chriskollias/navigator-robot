class Environment:
    def __init__(self, width=20, height=20):
        self.width = width
        self.height = height
        self.obstacles = []  # List of (x, y) obstacle positions
        
    def add_obstacle(self, x, y):
        """Add an obstacle at position (x, y)"""
        self.obstacles.append((x, y))
        
    def is_obstacle(self, x, y):
        """Check if position (x, y) contains an obstacle"""
        return (x, y) in self.obstacles
    
    def add_walls(self):
        """Add walls around the perimeter"""
        for x in range(self.width):
            self.add_obstacle(x, 0)
            self.add_obstacle(x, self.height - 1)
        
        for y in range(self.height):
            self.add_obstacle(0, y)
            self.add_obstacle(self.width - 1, y)
            
    def add_obstacle_pattern(self):
        """Add a simple pattern of obstacles"""
        # Add a diagonal line
        for i in range(5, 15):
            self.add_obstacle(i, i)
            
        # Add a square obstacle
        for x in range(3, 6):
            for y in range(12, 15):
                self.add_obstacle(x, y)
