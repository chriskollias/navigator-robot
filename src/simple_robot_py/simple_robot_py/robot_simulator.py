import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, UInt32
from .a_star import AStar
from .environment import Environment


class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_simulator')

        # QoS settings
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        
        # Publishers
        self.position_publisher = self.create_publisher(Pose2D, 'robot_position', 10)
        self.sensor_publisher = self.create_publisher(Float32MultiArray, 'robot_sensors', 10)
        self.map_publisher = self.create_publisher(Float32MultiArray, 'environment_map', qos_profile=map_qos)
        self.goal_publisher = self.create_publisher(Pose2D, 'robot_goal', qos_profile=map_qos)
        self.path_publisher = self.create_publisher(Float32MultiArray, 'robot_path', qos_profile=map_qos)
        self.step_publisher = self.create_publisher(UInt32, 'robot_steps', qos_profile=map_qos)

        # Create our environment
        self.env = Environment()
        self.env.add_walls()
        self.env.add_obstacle_pattern()
        
        # Robot state
        self.x = 12.0
        self.y = 9.0
        self.theta = 0.0
        self.speed = 0.5
        self.step_count = 0
        
        # Sensor setup
        self.n_sensors = 8  # 8 sensors around the robot
        self.max_sensor_range = 5.0

        # Initial goal state
        self.goal_x = 3.0
        self.goal_y = 2.0
        self.goal_reached_threshold = 1.0  # How close robot needs to be to goal

        # Setup path planner
        self.path_planner = None
        self.current_path = []
        self.path_index = 0
        self.recalculate_path_threshold = 3.0  # Distance before recalculating path

        # Setup our timer for simulation updates
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Publish the initial map
        self.publish_map()

        # Publish the initial goal
        self.publish_goal()

    def publish_map(self):
        """Publish the environment map"""
        map_msg = Float32MultiArray()
        map_data = []
        
        # Format: [width, height, obstacle_x1, obstacle_y1, obstacle_x2, obstacle_y2, ...]
        map_data.append(float(self.env.width))
        map_data.append(float(self.env.height))
        
        for obstacle in self.env.obstacles:
            map_data.append(float(obstacle[0]))
            map_data.append(float(obstacle[1]))
            
        map_msg.data = map_data
        self.map_publisher.publish(map_msg)
        self.get_logger().info(f'Published map with {len(self.env.obstacles)} obstacles')

    def get_sensor_readings(self):
        """Simulate distance sensor readings"""
        readings = []
        
        for i in range(self.n_sensors):
            # Calculate sensor angle
            sensor_angle = self.theta + (i * 2 * math.pi / self.n_sensors)
            
            # Check distances along the sensor ray
            reading = self.max_sensor_range
            for distance in [d * 0.5 for d in range(1, int(self.max_sensor_range * 2) + 1)]:
                sensor_x = self.x + distance * math.cos(sensor_angle)
                sensor_y = self.y + distance * math.sin(sensor_angle)
                
                # Convert to grid coordinates
                grid_x = int(round(sensor_x))
                grid_y = int(round(sensor_y))
                
                # Check if we hit an obstacle
                if (0 <= grid_x < self.env.width and 0 <= grid_y < self.env.height and 
                        self.env.is_obstacle(grid_x, grid_y)):
                    reading = distance
                    break
            
            readings.append(reading)
            
        return readings

    def publish_goal(self):
        """Publish the robot's current goal"""
        goal_msg = Pose2D()
        goal_msg.x = self.goal_x
        goal_msg.y = self.goal_y
        goal_msg.theta = 0.0  # Orientation doesn't matter for the goal
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f'Goal published at ({self.goal_x}, {self.goal_y})')

    def set_new_goal(self):
        """Set a new random goal for the robot"""
        # Ensure goal isn't in an obstacle
        valid_goal = False
        while not valid_goal:
            # Keep goal away from walls
            self.goal_x = random.uniform(2.0, self.env.width - 3.0)
            self.goal_y = random.uniform(2.0, self.env.height - 3.0)
            
            # Check if goal is in obstacle
            grid_x = int(round(self.goal_x))
            grid_y = int(round(self.goal_y))
            if not self.env.is_obstacle(grid_x, grid_y):
                valid_goal = True
        
        self.publish_goal()

    def goal_seeking_direction(self):
        """Calculate the direction to the goal"""
        # Calculate vector to goal
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        
        # Calculate angle to goal
        angle_to_goal = math.atan2(dy, dx)
        
        # Calculate difference between current orientation and goal direction
        angle_diff = angle_to_goal - self.theta
        
        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        return angle_diff

    def distance_to_goal(self):
        """Calculate distance to the goal"""
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        return math.sqrt(dx*dx + dy*dy)

    def initialize_path_planner(self):
        """Initialize the A* path planner with current environment"""
        self.path_planner = AStar(self.env.width, self.env.height, self.env.obstacles)
        
    def calculate_path_to_goal(self):
        """Calculate a path to the goal using A*"""
        # Initialize planner if needed
        if self.path_planner is None:
            self.initialize_path_planner()
        
        # Find path from current position to goal
        start = (self.x, self.y)
        goal = (self.goal_x, self.goal_y)
        
        self.current_path = self.path_planner.find_path(start, goal)
        self.path_index = 0
        
        if self.current_path:
            self.get_logger().info(f"Path found with {len(self.current_path)} steps")
            self.get_logger().info(f"Path: {self.current_path}")  # Log the actual path
        else:
            self.get_logger().warn(f"No path found from {start} to {goal}")

    def publish_path(self):
        """Publish the current path as a message"""
        path_msg = Float32MultiArray()
        
        if self.current_path:
            # Flatten the path points into a list
            path_data = []
            for point in self.current_path:
                path_data.append(float(point[0]))
                path_data.append(float(point[1]))
            
            path_msg.data = path_data
            self.path_publisher.publish(path_msg)

    def combined_navigation(self, sensor_readings, angle_to_goal):
        """Combine obstacle avoidance with goal seeking"""
        # Check front sensors for obstacles
        front_sensor = sensor_readings[0]
        front_left = sensor_readings[7]
        front_right = sensor_readings[1]
        
        # Start with goal-seeking direction
        new_theta = self.theta + angle_to_goal * 0.3  # Gradual turning
        
        # If obstacle is close, prioritize obstacle avoidance
        if front_sensor < 1.5:
            # Decide which way to turn based on goal direction
            if angle_to_goal > 0:
                # Goal is to the left, try to turn left
                if front_left > 1.5:
                    new_theta = self.theta + 0.3
                else:
                    new_theta = self.theta - 0.3
            else:
                # Goal is to the right, try to turn right
                if front_right > 1.5:
                    new_theta = self.theta - 0.3
                else:
                    new_theta = self.theta + 0.3
        elif front_left < 1.0:
            # Obstacle on front-left, turn right
            new_theta = self.theta - 0.2
        elif front_right < 1.0:
            # Obstacle on front-right, turn left
            new_theta = self.theta + 0.2
        
        return new_theta

    def handle_goal_reached(self):
        """Handle logic when robot reaches its current goal"""
        self.get_logger().info('Goal reached! Setting new goal.')
        
        # Clear the current path
        self.current_path = []
        self.path_index = 0

        # Reset step counter
        self.step_count = 0
        
        # Set a new goal
        self.set_new_goal()

    def timer_callback(self):
        # Get sensor readings
        sensor_readings = self.get_sensor_readings()

        # Check if goal is reached
        if self.distance_to_goal() < self.goal_reached_threshold:
            self.handle_goal_reached()

        # Calculate or recalculate path if needed
        if not self.current_path or self.path_index >= len(self.current_path):
            self.calculate_path_to_goal()
            self.publish_path()  # Publish the new path
        
        # Path following logic
        if self.current_path and len(self.current_path) > self.path_index:
            # Get next path point
            next_point = self.current_path[self.path_index]
            
            # Calculate direction to next point
            dx = next_point[0] - self.x
            dy = next_point[1] - self.y
            
            # Check if we've reached the current waypoint
            dist_to_waypoint = math.sqrt(dx*dx + dy*dy)
            if dist_to_waypoint < 0.5:  # Close enough to this waypoint
                self.path_index += 1  # Move to next waypoint
                
                # If we've reached the end of the path but not the goal, recalculate
                if self.path_index >= len(self.current_path) and self.distance_to_goal() > self.goal_reached_threshold:
                    self.calculate_path_to_goal()
                    self.publish_path()  # Publish the new path
            
            # Calculate angle to the waypoint
            angle_to_point = math.atan2(dy, dx)
            
            # Calculate difference between current orientation and goal direction
            angle_diff = angle_to_point - self.theta
            
            # Normalize angle difference to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            self.theta = angle_to_point       
        
        else:
            # Fallback to direct goal seeking if no path
            angle_to_goal = self.goal_seeking_direction()
            self.theta = self.combined_navigation(sensor_readings, angle_to_goal)

        # Move the robot
        new_x = self.x + self.speed * math.cos(self.theta)
        new_y = self.y + self.speed * math.sin(self.theta)

        # Check if new position is valid, if true take step
        grid_x = int(round(new_x))
        grid_y = int(round(new_y))

        if (0 <= grid_x < self.env.width and 0 <= grid_y < self.env.height and 
                not self.env.is_obstacle(grid_x, grid_y)):
            self.x = new_x
            self.y = new_y
            self.step_count += 1  # Increment step counter
        else:
            # If invalid, turn around
            self.theta += math.pi / 2

        # Publish robot position
        pos_msg = Pose2D()
        pos_msg.x = self.x
        pos_msg.y = self.y
        pos_msg.theta = self.theta
        self.position_publisher.publish(pos_msg)
        
        # Publish sensor data
        sensor_msg = Float32MultiArray()
        sensor_msg.data = sensor_readings
        self.sensor_publisher.publish(sensor_msg)

        # Publish step count
        step_msg = UInt32()
        step_msg.data = self.step_count
        self.step_publisher.publish(step_msg)

        # Recalculate distance to goal in case new goal was set
        distance_to_goal = self.distance_to_goal()

        # Logging
        self.get_logger().info(f'Robot position: x: {self.x:.2f}, y: {self.y:.2f}, theta: {self.theta:.2f}, steps: {self.step_count}, distance to goal: {distance_to_goal:.2f}')


def main(args=None):
    rclpy.init(args=args)
    robot_simulator = RobotSimulator()
    rclpy.spin(robot_simulator)
    robot_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
