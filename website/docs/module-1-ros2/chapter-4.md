---
title: Chapter 4 - Navigation and Path Planning
description: Implementing navigation systems and path planning algorithms for mobile robots
keywords: [navigation, path planning, slam, localization, mapping]
sidebar_position: 5
module_ref: module-1-ros2
prerequisites: ["Module 1, Chapter 3"]
learning_objectives: ["Implement navigation systems for mobile robots", "Plan collision-free paths in known and unknown environments", "Integrate perception with navigation", "Evaluate navigation performance"]
estimated_reading_time: 60
exercises_count: 3
---

# Chapter 4: Navigation and Path Planning

## Learning Objectives
- Implement navigation systems for mobile robots using ROS 2
- Plan collision-free paths in known and unknown environments
- Integrate perception systems with navigation
- Evaluate navigation performance and optimize parameters

## Prerequisites
- Understanding of ROS 2 communication patterns
- Knowledge of coordinate frames and transformations
- Experience with sensor data processing (LIDAR, cameras)
- Basic understanding of graph algorithms and optimization

## Core Concepts

Navigation in robotics involves enabling a robot to move from a starting location to a goal location while avoiding obstacles. This requires several interconnected components:

### Localization
Determining the robot's position and orientation in its environment, often using sensor data and maps.

### Mapping
Creating or using representations of the environment, which can be static (pre-built) or dynamically updated.

### Path Planning
Computing a sequence of waypoints or a continuous path from start to goal that avoids obstacles.

### Path Execution
Following the planned path using robot control systems while handling deviations and dynamic obstacles.

### Navigation Stack Components
The ROS 2 navigation stack includes:
- **AMCL (Adaptive Monte Carlo Localization)**: Probabilistic localization
- **Costmap 2D**: Obstacle representation and inflation
- **Global Planner**: Computing optimal paths in static maps
- **Local Planner**: Executing paths while avoiding dynamic obstacles
- **Controller**: Sending velocity commands to robot base

## Implementation

### Creating a Navigation Node

Here's an implementation of a basic navigation node that interfaces with the ROS 2 Navigation stack:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
import math
from typing import List, Tuple


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10)
        
        # TF listener for transforming coordinates
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Robot state
        self.current_pose = None
        self.current_scan = None
        self.latest_path = None
        self.navigation_goal = None
        self.navigation_active = False
        
        # Navigation parameters
        self.linear_vel = 0.5  # m/s
        self.angular_vel = 0.5  # rad/s
        self.arrival_threshold = 0.5  # meters
        self.rotation_threshold = 0.1  # radians
        
        # Create navigation timer
        self.nav_timer = self.create_timer(0.1, self.navigation_callback)
        
        # Create goal setting timer for demonstration
        self.goal_timer = self.create_timer(10.0, self.set_navigation_goal)
        
        self.get_logger().info('Navigation Node initialized')

    def odom_callback(self, msg):
        """
        Callback for odometry updates
        """
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """
        Callback for laser scan updates
        """
        self.current_scan = msg

    def path_callback(self, msg):
        """
        Callback for path updates
        """
        self.latest_path = msg

    def set_navigation_goal(self):
        """
        Set a navigation goal (for demonstration purposes)
        """
        if not self.current_pose:
            self.get_logger().warn('Cannot set goal without current pose')
            return
        
        # Create a goal offset from current position (for demonstration)
        goal = PoseStamped()
        goal.header = Header()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        
        # Set goal 2 meters ahead of current position in robot's direction
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        goal.pose.position.x = self.current_pose.position.x + 2.0 * math.cos(current_yaw)
        goal.pose.position.y = self.current_pose.position.y + 2.0 * math.sin(current_yaw)
        goal.pose.position.z = self.current_pose.position.z
        
        # Set goal orientation to face the same direction
        goal.pose.orientation = self.current_pose.orientation
        
        self.navigation_goal = goal.pose
        self.navigation_active = True
        
        # Publish the goal to the navigation system
        goal_stamped = PoseStamped()
        goal_stamped.header = goal.header
        goal_stamped.pose = goal.pose
        self.goal_pub.publish(goal_stamped)
        
        self.get_logger().info(f'Set navigation goal: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})')

    def navigation_callback(self):
        """
        Main navigation control callback
        """
        if not self.navigation_active or not self.current_pose:
            return
        
        if not self.navigation_goal:
            self.get_logger().warn('Navigation active but no goal set')
            return
        
        # Calculate distance to goal
        dx = self.navigation_goal.position.x - self.current_pose.position.x
        dy = self.navigation_goal.position.y - self.current_pose.position.y
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        
        # Check if we've reached the goal
        if distance_to_goal < self.arrival_threshold:
            self.get_logger().info('Reached goal position!')
            self.navigation_active = False
            self.publish_status('Goal reached')
            return
        
        # Calculate required orientation to face goal
        required_yaw = math.atan2(dy, dx)
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Normalize angle difference
        angle_diff = required_yaw - current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Create command to move toward goal
        cmd_vel = self.calculate_navigation_command(angle_diff, distance_to_goal)
        
        # For this example, we'll just log the command
        # In a real implementation, this would go to a velocity controller
        self.get_logger().debug(f'Navigation command: linear={cmd_vel[0]:.3f}, angular={cmd_vel[1]:.3f}')

    def calculate_navigation_command(self, angle_diff, distance_to_goal):
        """
        Calculate navigation command based on angle difference and distance
        """
        linear_vel = 0.0
        angular_vel = 0.0
        
        # If we need to rotate significantly, do that first
        if abs(angle_diff) > self.rotation_threshold:
            angular_vel = np.clip(angle_diff * 1.0, -self.angular_vel, self.angular_vel)
        else:
            # When roughly aligned, move forward
            linear_vel = min(self.linear_vel, distance_to_goal * 0.5)  # Slow down as we approach
            # Add small correction for remaining angle error
            angular_vel = np.clip(angle_diff * 0.5, -self.angular_vel, self.angular_vel)
        
        return (linear_vel, angular_vel)

    def get_yaw_from_quaternion(self, orientation):
        """
        Extract yaw angle from quaternion
        """
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_status(self, status_text):
        """
        Publish navigation status
        """
        status_msg = String()
        status_msg.data = status_text
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    nav_node = NavigationNode()
    
    try:
        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        nav_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Path Planning Implementation

Here's an implementation of A* path planning algorithm for grid-based navigation:

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
import numpy as np
import heapq
from typing import List, Tuple, Optional


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/computed_path', 10)
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        # Internal state
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin = None
        
        # For demonstration, we'll plan a path when a map is received
        self.map_received = False
        
        self.get_logger().info('Path Planner Node initialized')

    def map_callback(self, msg):
        """
        Callback for receiving the occupancy grid map
        """
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        if not self.map_received:
            self.map_received = True
            # Plan a path once we have the map (for demonstration)
            self.plan_path()

    def plan_path(self):
        """
        Plan a path using A* algorithm
        """
        if self.map_data is None:
            self.get_logger().error('Cannot plan path: no map data available')
            return
        
        # Define start and goal positions (for demonstration)
        start = (50, 50)  # Grid coordinates (x, y)
        goal = (int(self.map_width - 50), int(self.map_height - 50))
        
        # Make sure start and goal are within bounds and not occupied
        start = self.clamp_to_bounds(start)
        goal = self.clamp_to_bounds(goal)
        
        if self.is_occupied(start) or self.is_occupied(goal):
            self.get_logger().error('Start or goal is in occupied space')
            return
        
        path = self.a_star(start, goal)
        
        if path:
            self.publish_path(path)
            self.get_logger().info(f'Path found with {len(path)} waypoints')
        else:
            self.get_logger().error('No path found from start to goal')

    def clamp_to_bounds(self, pos: Tuple[int, int]) -> Tuple[int, int]:
        """
        Clamp position to map bounds
        """
        x, y = pos
        x = max(0, min(x, self.map_width - 1))
        y = max(0, min(y, self.map_height - 1))
        return (x, y)

    def is_occupied(self, pos: Tuple[int, int]) -> bool:
        """
        Check if a grid cell is occupied
        """
        x, y = pos
        if x < 0 or x >= self.map_width or y < 0 or y >= self.map_height:
            return True  # Out of bounds is considered occupied
        
        # Occupancy values: 0 = free, 100 = occupied, -1 = unknown
        return self.map_data[y, x] >= 90  # Consider >90% as occupied

    def a_star(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        A* path planning algorithm
        """
        # Define heuristic function (Euclidean distance)
        def heuristic(pos):
            return math.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)
        
        # Initialize data structures
        open_set = [(0, start)]  # Priority queue: (f_score, position)
        heapq.heapify(open_set)
        
        came_from = {}  # For path reconstruction
        g_score = {start: 0}  # Cost from start to position
        f_score = {start: heuristic(start)}  # Estimated total cost
        
        # 8-directional movement (for more detailed paths)
        neighbors = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]
        
        while open_set:
            # Get the node with lowest f_score
            current_f, current = heapq.heappop(open_set)
            
            # Check if we've reached the goal
            if current == goal:
                return self.reconstruct_path(came_from, current)
            
            # Check all neighbors
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Skip if outside map bounds
                if neighbor[0] < 0 or neighbor[0] >= self.map_width or \
                   neighbor[1] < 0 or neighbor[1] >= self.map_height:
                    continue
                
                # Skip if occupied
                if self.is_occupied(neighbor):
                    continue
                
                # Calculate tentative g_score
                movement_cost = math.sqrt(dx*dx + dy*dy)  # Euclidean distance between cells
                tentative_g_score = g_score[current] + movement_cost
                
                # If this path to neighbor is better than any previous one
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # This path is the best until now, record it
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor)
                    
                    # Add to open set if not already there
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found
        return None

    def reconstruct_path(self, came_from: dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Reconstruct the path from start to goal
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        
        path.reverse()
        return path

    def publish_path(self, grid_path: List[Tuple[int, int]]):
        """
        Convert grid path to world coordinates and publish as Path message
        """
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for grid_x, grid_y in grid_path:
            # Convert grid coordinates to world coordinates
            world_x = self.map_origin[0] + (grid_x + 0.5) * self.map_resolution
            world_y = self.map_origin[1] + (grid_y + 0.5) * self.map_resolution
            
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = world_x
            pose_stamped.pose.position.y = world_y
            pose_stamped.pose.position.z = 0.0
            
            # Set orientation to face along the path (simplified)
            if len(grid_path) > 1:
                next_idx = min(grid_path.index((grid_x, grid_y)) + 1, len(grid_path) - 1)
                next_grid_x, next_grid_y = grid_path[next_idx]
                
                # Calculate direction to next point
                dx = next_grid_x - grid_x
                dy = next_grid_y - grid_y
                yaw = math.atan2(dy, dx)
                
                # Convert yaw to quaternion
                from tf_transformations import quaternion_from_euler
                quat = quaternion_from_euler(0, 0, yaw)
                pose_stamped.pose.orientation.x = quat[0]
                pose_stamped.pose.orientation.y = quat[1]
                pose_stamped.pose.orientation.z = quat[2]
                pose_stamped.pose.orientation.w = quat[3]
            
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    planner_node = PathPlannerNode()
    
    try:
        rclpy.spin(planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Implementing Navigation with Costmaps

Here's an example of working with costmaps for navigation:

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
import numpy as np
from typing import List, Tuple


class CostmapNode(Node):
    def __init__(self):
        super().__init__('costmap_node')
        
        # Publishers
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/local_costmap', 10)
        self.collision_pub = self.create_publisher(PointStamped, '/collision_warning', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Internal state
        self.current_pose = None
        self.current_scan = None
        
        # Costmap parameters
        self.costmap_size = 20  # meters
        self.resolution = 0.1   # meters per cell
        self.costmap_width = int(self.costmap_size / self.resolution)
        self.costmap_height = int(self.costmap_size / self.resolution)
        
        # Create costmap timer
        self.costmap_timer = self.create_timer(0.5, self.update_costmap)
        
        self.get_logger().info('Costmap Node initialized')

    def odom_callback(self, msg):
        """
        Callback for odometry updates
        """
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """
        Callback for laser scan updates
        """
        self.current_scan = msg

    def update_costmap(self):
        """
        Update the local costmap based on laser scan data
        """
        if not self.current_scan or not self.current_pose:
            return
        
        # Create a new costmap
        costmap = np.zeros((self.costmap_height, self.costmap_width), dtype=np.int8)
        
        # Convert robot position to costmap coordinates
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Process laser scan data to populate costmap
        for i, range_val in enumerate(self.current_scan.ranges):
            if not (self.current_scan.range_min <= range_val <= self.current_scan.range_max):
                continue  # Invalid range reading
            
            # Calculate angle of this reading
            angle = self.current_scan.angle_min + i * self.current_scan.angle_increment
            
            # Calculate the world coordinates of the obstacle
            world_x = robot_x + range_val * math.cos(angle)
            world_y = robot_y + range_val * math.sin(angle)
            
            # Convert to costmap coordinates
            costmap_x = int((world_x - (robot_x - self.costmap_size/2)) / self.resolution)
            costmap_y = int((world_y - (robot_y - self.costmap_size/2)) / self.resolution)
            
            # Check if coordinates are within bounds
            if 0 <= costmap_x < self.costmap_width and 0 <= costmap_y < self.costmap_height:
                # Set this cell as occupied (value 100)
                costmap[costmap_y, costmap_x] = 100
        
        # Publish the costmap
        costmap_msg = OccupancyGrid()
        costmap_msg.header = Header()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = 'odom'  # Costmap is in robot's local frame
        
        costmap_msg.info.resolution = self.resolution
        costmap_msg.info.width = self.costmap_width
        costmap_msg.info.height = self.costmap_height
        
        # Set origin to robot's current position
        costmap_msg.info.origin.position.x = robot_x - self.costmap_size / 2
        costmap_msg.info.origin.position.y = robot_y - self.costmap_size / 2
        costmap_msg.info.origin.position.z = 0.0
        costmap_msg.info.origin.orientation.w = 1.0
        
        # Flatten the costmap for the message
        costmap_msg.data = costmap.flatten().tolist()
        
        self.costmap_pub.publish(costmap_msg)
        
        # Check for imminent collision (based on scan data)
        self.check_collision_risk()

    def check_collision_risk(self):
        """
        Check if there's an imminent collision risk based on laser scan
        """
        if not self.current_scan:
            return
        
        # Check for obstacles in front of the robot
        front_scan_start = len(self.current_scan.ranges) // 2 - 10
        front_scan_end = len(self.current_scan.ranges) // 2 + 10
        
        min_distance = float('inf')
        for i in range(front_scan_start, front_scan_end):
            if i >= 0 and i < len(self.current_scan.ranges):
                if self.current_scan.range_min <= self.current_scan.ranges[i] <= self.current_scan.range_max:
                    min_distance = min(min_distance, self.current_scan.ranges[i])
        
        # If obstacle is very close, publish a warning
        collision_threshold = 0.5  # meters
        if min_distance < collision_threshold:
            warning = PointStamped()
            warning.header = Header()
            warning.header.stamp = self.get_clock().now().to_msg()
            warning.header.frame_id = 'base_link'
            warning.point.x = min_distance
            warning.point.y = 0.0
            warning.point.z = 0.0
            
            self.collision_pub.publish(warning)
            self.get_logger().warn(f'Collision warning: obstacle at {min_distance:.2f}m ahead')


def main(args=None):
    rclpy.init(args=args)
    costmap_node = CostmapNode()
    
    try:
        rclpy.spin(costmap_node)
    except KeyboardInterrupt:
        pass
    finally:
        costmap_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercises

1. **Logical Exercise**: Compare different path planning algorithms (A*, Dijkstra, RRT). In what scenarios would you choose each algorithm for humanoid robot navigation?

2. **Conceptual Exercise**: Explain the difference between global and local path planning. How do these two approaches work together in a complete navigation system?

3. **Implementation Exercise**: Create a complete navigation system that incorporates both global path planning and local obstacle avoidance. Implement a system that can navigate to a goal while avoiding dynamic obstacles detected by sensors.

## Summary

This chapter covered navigation and path planning for mobile robots, including localization, mapping, path planning algorithms, and path execution. We explored the components of the ROS 2 navigation stack and implemented examples of path planning algorithms and costmap management.

Effective navigation is essential for mobile robots to operate autonomously in complex environments. Understanding how to plan paths, manage maps, and execute navigation safely enables robots to move purposefully while avoiding obstacles.

## References
- [Navigation2 Documentation](https://navigation.ros.org/)
- [Probabilistic Robotics by Sebastian Thrun](https://mitpress.mit.edu/books/probabilistic-robotics)
- [SLAM in Robotics](https://ieeexplore.ieee.org/document/6343147)