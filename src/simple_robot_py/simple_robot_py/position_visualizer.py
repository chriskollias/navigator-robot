import os
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, UInt32


class PositionVisualizer(Node):
    def __init__(self):
        super().__init__('position_visualizer')

        # QoS settings
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        
        # Initialize map and robot state
        self.map_width = 20
        self.map_height = 20
        self.obstacles = []
        self.robot_x = 10.0
        self.robot_y = 10.0
        self.robot_theta = 0.0
        self.sensor_readings = []
        self.step_count = 0
        
        # Add goal state
        self.goal_x = None
        self.goal_y = None

        # Subscriptions
        self.position_sub = self.create_subscription(
            Pose2D,
            'robot_position',
            self.position_callback,
            10)
        
        self.sensor_sub = self.create_subscription(
            Float32MultiArray,
            'robot_sensors',
            self.sensor_callback,
            10)
            
        self.map_sub = self.create_subscription(
            Float32MultiArray,
            'environment_map',
            self.map_callback,
            qos_profile=map_qos)
        
        self.goal_sub = self.create_subscription(
            Pose2D,
            'robot_goal',
            self.goal_callback,
            qos_profile=map_qos)
        self.step_sub = self.create_subscription(
            UInt32,
            'robot_steps',
            self.step_callback,
            qos_profile=map_qos)
        
        # Timer for visualization updates
        self.timer = self.create_timer(0.1, self.visualize)
    
    def position_callback(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_theta = msg.theta
    
    def sensor_callback(self, msg):
        self.sensor_readings = msg.data
    
    def map_callback(self, msg):

        if len(msg.data) >= 2:
            self.map_width = int(msg.data[0])
            self.map_height = int(msg.data[1])
            
            # Extract obstacle positions
            self.obstacles = []
            for i in range(2, len(msg.data), 2):
                if i + 1 < len(msg.data):
                    self.obstacles.append((int(msg.data[i]), int(msg.data[i + 1])))
    
    def goal_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y

    def step_callback(self, msg):
        self.step_count = msg.data

    def visualize(self):
        # Clear console
        os.system('clear' if os.name == 'posix' else 'cls')
        
        # Create a grid
        grid = [[' ' for _ in range(self.map_width)] for _ in range(self.map_height)]
        
        # Add obstacles to grid
        for x, y in self.obstacles:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                grid[y][x] = '#'
        
        # Add robot to grid
        robot_grid_x = int(round(self.robot_x))
        robot_grid_y = int(round(self.robot_y))
        
        if 0 <= robot_grid_x < self.map_width and 0 <= robot_grid_y < self.map_height:
            grid[robot_grid_y][robot_grid_x] = 'R'
        
        # Add goal to grid if available
        if self.goal_x is not None and self.goal_y is not None:
            goal_grid_x = int(round(self.goal_x))
            goal_grid_y = int(round(self.goal_y))
            
            if 0 <= goal_grid_x < self.map_width and 0 <= goal_grid_y < self.map_height:
                grid[goal_grid_y][goal_grid_x] = 'G'

        # Add sensor readings if available
        if self.sensor_readings:
            for i, reading in enumerate(self.sensor_readings):
                if reading < 5.0:  # Only show nearby readings
                    # Calculate sensor endpoint
                    sensor_angle = self.robot_theta + (i * 2 * math.pi / len(self.sensor_readings))
                    sensor_x = self.robot_x + reading * math.cos(sensor_angle)
                    sensor_y = self.robot_y + reading * math.sin(sensor_angle)
                    
                    # Convert to grid
                    sensor_grid_x = int(round(sensor_x))
                    sensor_grid_y = int(round(sensor_y))
                    
                    # Mark sensor endpoint
                    if (0 <= sensor_grid_x < self.map_width and 
                            0 <= sensor_grid_y < self.map_height and
                            grid[sensor_grid_y][sensor_grid_x] == ' '):
                        grid[sensor_grid_y][sensor_grid_x] = '+'

        # After adding sensors, add this to show path to goal:
        if self.goal_x is not None and self.goal_y is not None:
            # Draw a simple line to the goal
            dx = self.goal_x - self.robot_x
            dy = self.goal_y - self.robot_y
            steps = int(max(abs(dx), abs(dy)) * 2)
            if steps > 0:
                for i in range(1, steps):
                    path_x = self.robot_x + dx * i / steps
                    path_y = self.robot_y + dy * i / steps
                    
                    path_grid_x = int(round(path_x))
                    path_grid_y = int(round(path_y))
                    
                    if (0 <= path_grid_x < self.map_width and 
                            0 <= path_grid_y < self.map_height and
                            grid[path_grid_y][path_grid_x] == ' '):
                        grid[path_grid_y][path_grid_x] = '·'

        # Draw the grid
        print('-' * (self.map_width * 2 + 1))
        for row in grid:
            print('|' + ' '.join(row) + '|')
        print('-' * (self.map_width * 2 + 1))
        
        # Show robot information
        print(f"Robot Position: x={self.robot_x:.2f}, y={self.robot_y:.2f}, θ={self.robot_theta:.2f}")
        print(f"Step Count: {self.step_count}")

        # Show sensor readings if available
        if self.sensor_readings:
            sensor_str = "Sensors: "
            for i, reading in enumerate(self.sensor_readings):
                # Only display a few sensors for readability
                if i % 2 == 0:
                    angle_deg = (i * 360) // len(self.sensor_readings)
                    sensor_str += f"{angle_deg}°:{reading:.1f}m "
            print(sensor_str)

        # Add after drawing the grid and robot information:
        # Show goal information if available
        if self.goal_x is not None and self.goal_y is not None:
            dx = self.goal_x - self.robot_x
            dy = self.goal_y - self.robot_y
            distance = math.sqrt(dx*dx + dy*dy)
            print(f"Goal Position: x={self.goal_x:.2f}, y={self.goal_y:.2f}, Distance: {distance:.2f}")

        # Add a legend
        print("Legend: R=Robot, G=Goal, #=Obstacle, +=Sensor Reading, ·=Path")


def main(args=None):
    rclpy.init(args=args)
    position_visualizer = PositionVisualizer()
    rclpy.spin(position_visualizer)
    position_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
