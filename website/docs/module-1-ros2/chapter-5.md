---
title: Chapter 5 - Multi-Robot Systems and Coordination
description: Designing systems for coordinating multiple robots and addressing multi-robot challenges
keywords: [multi-robot, coordination, distributed, communication, teamwork]
sidebar_position: 6
module_ref: module-1-ros2
prerequisites: ["Module 1, Chapter 4"]
learning_objectives: ["Design systems for coordinating multiple robots", "Implement communication protocols between robots", "Address challenges in multi-robot coordination", "Evaluate multi-robot system performance"]
estimated_reading_time: 60
exercises_count: 3
---

# Chapter 5: Multi-Robot Systems and Coordination

## Learning Objectives
- Design systems for coordinating multiple robots in shared environments
- Implement communication protocols between robots
- Address challenges in multi-robot coordination such as task allocation and conflict resolution
- Evaluate performance of multi-robot systems

## Prerequisites
- Understanding of ROS 2 communication patterns (topics, services, actions)
- Knowledge of distributed systems concepts
- Experience with robot navigation and path planning
- Familiarity with state estimation and localization

## Core Concepts

Multi-robot systems involve multiple autonomous agents working together to achieve common goals. These systems offer several advantages over single robots, including:

### Advantages of Multi-Robot Systems
- **Redundancy**: System can continue operating if individual robots fail
- **Parallelism**: Multiple tasks can be performed simultaneously
- **Scalability**: System performance can improve with more robots
- **Flexibility**: Robots can specialize in different tasks

### Coordination Approaches
Multi-robot coordination can be achieved through various approaches:

1. **Centralized Coordination**: A central authority plans and coordinates all robots
2. **Decentralized Coordination**: Each robot makes local decisions based on shared information
3. **Hybrid Coordination**: Combines centralized and decentralized elements

### Communication Protocols
- **Broadcast Communication**: Robots broadcast their states to all others
- **Point-to-Point Communication**: Direct robot-to-robot communication
- **Server-Based Communication**: Robots communicate through shared servers/services

### Key Challenges
- **Task Allocation**: Assigning tasks to appropriate robots
- **Path Planning**: Planning paths that avoid collisions between robots
- **Resource Conflicts**: Managing shared resources or space
- **Communication Limitations**: Dealing with latency, bandwidth, and reliability issues

## Implementation

### Multi-Robot Communication Framework

Here's an implementation of a basic multi-robot communication framework:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from multirobot_msgs.msg import RobotState, RobotCommand
import json
import uuid
from typing import Dict, List


class MultiRobotCommunicator(Node):
    def __init__(self):
        super().__init__('multirobot_communicator')
        
        # Generate unique robot ID
        self.robot_id = str(uuid.uuid4())[:8]  # Short ID for readability
        self.get_logger().info(f'Robot ID: {self.robot_id}')
        
        # Publishers for communication
        self.state_pub = self.create_publisher(RobotState, '/multirobot_state', 10)
        self.broadcast_pub = self.create_publisher(String, '/multirobot_broadcast', 10)
        self.cmd_pub = self.create_publisher(RobotCommand, '/multirobot_command', 10)
        
        # Subscribers for communication
        self.state_sub = self.create_subscription(
            RobotState,
            '/multirobot_state',
            self.robot_state_callback,
            10)
        
        self.broadcast_sub = self.create_subscription(
            String,
            '/multirobot_broadcast',
            self.broadcast_callback,
            10)
        
        self.cmd_sub = self.create_subscription(
            RobotCommand,
            '/multirobot_command',
            self.command_callback,
            10)
        
        # Track other robots' states
        self.other_robots: Dict[str, RobotState] = {}
        self.last_update_times: Dict[str, float] = {}
        
        # Robot state
        self.current_pose = None
        self.current_task = None
        self.robot_status = "idle"
        
        # Create communication timer
        self.comm_timer = self.create_timer(1.0, self.communicate_state)
        
        # Create coordination timer
        self.coordination_timer = self.create_timer(2.0, self.coordination_loop)
        
        self.get_logger().info('Multi-Robot Communicator initialized')

    def robot_state_callback(self, msg: RobotState):
        """
        Callback for receiving state from other robots
        """
        if msg.robot_id != self.robot_id:
            # Update the state of the other robot
            self.other_robots[msg.robot_id] = msg
            self.last_update_times[msg.robot_id] = self.get_clock().now().nanoseconds / 1e9
        
        # Log the received state for debugging
        self.get_logger().debug(f'Received state from robot {msg.robot_id}: pos=({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def broadcast_callback(self, msg: String):
        """
        Callback for receiving broadcast messages
        """
        try:
            data = json.loads(msg.data)
            sender_id = data.get('sender_id')
            
            if sender_id != self.robot_id:
                self.get_logger().debug(f'Received broadcast from {sender_id}: {data.get("message")}')
                
                # Handle different types of broadcast messages
                msg_type = data.get('type')
                if msg_type == 'task_announcement':
                    self.handle_task_announcement(data)
                elif msg_type == 'resource_request':
                    self.handle_resource_request(data)
        
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid JSON in broadcast message')

    def command_callback(self, msg: RobotCommand):
        """
        Callback for receiving commands from coordination system
        """
        if msg.target_robot_id == self.robot_id:
            self.get_logger().info(f'Received command: {msg.command_type} - {msg.command_details}')
            self.execute_command(msg)

    def communicate_state(self):
        """
        Publish the current robot state to other robots
        """
        state_msg = RobotState()
        state_msg.robot_id = self.robot_id
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = 'map'
        
        # Set pose (if available)
        if self.current_pose:
            state_msg.pose = self.current_pose
        else:
            # Set default pose if not available
            state_msg.pose.position.x = 0.0
            state_msg.pose.position.y = 0.0
            state_msg.pose.position.z = 0.0
            state_msg.pose.orientation.w = 1.0
        
        # Set robot status
        state_msg.status = self.robot_status
        state_msg.current_task = self.current_task or "none"
        
        # Set robot capabilities (example)
        state_msg.capabilities = ["navigation", "manipulation", "communication"]
        
        self.state_pub.publish(state_msg)

    def broadcast_message(self, message_type: str, message_content: Dict):
        """
        Broadcast a message to all robots
        """
        message_content['type'] = message_type
        message_content['sender_id'] = self.robot_id
        message_content['timestamp'] = self.get_clock().now().nanoseconds / 1e9
        
        msg = String()
        msg.data = json.dumps(message_content)
        self.broadcast_pub.publish(msg)

    def handle_task_announcement(self, data: Dict):
        """
        Handle announcement of a new task
        """
        task_id = data.get('task_id')
        task_location = data.get('location')
        task_type = data.get('task_type')
        
        self.get_logger().info(f'New task announced: {task_id} at {task_location} ({task_type})')
        
        # In a real system, this would trigger a task allocation process
        # For this example, we'll just check if we should volunteer
        if self.should_volunteer_for_task(task_type, task_location):
            self.volunteer_for_task(task_id)

    def handle_resource_request(self, data: Dict):
        """
        Handle a request for resource sharing
        """
        requester_id = data.get('requester_id')
        resource_type = data.get('resource_type')
        resource_location = data.get('location')
        
        self.get_logger().info(f'Resource request from {requester_id}: {resource_type} at {resource_location}')
        
        # Decide whether to respond to the request
        if self.can_provide_resource(resource_type, resource_location):
            self.respond_to_resource_request(requester_id, resource_type)

    def should_volunteer_for_task(self, task_type: str, task_location: Dict) -> bool:
        """
        Determine if this robot should volunteer for a task
        """
        # Simple heuristic: volunteer if the task is close and we can perform it
        if self.current_pose and 'capabilities' in locals():
            task_x = task_location.get('x', 0)
            task_y = task_location.get('y', 0)
            
            distance = ((task_x - self.current_pose.position.x)**2 + 
                       (task_y - self.current_pose.position.y)**2)**0.5
            
            # Volunteer if task is within 5 meters and we're not busy
            return distance < 5.0 and self.robot_status == 'idle'
        
        return False

    def volunteer_for_task(self, task_id: str):
        """
        Volunteer to perform a task
        """
        self.get_logger().info(f'Volunteering for task {task_id}')
        
        # Send a bid message to the task coordinator
        bid_message = {
            'type': 'task_bid',
            'task_id': task_id,
            'robot_id': self.robot_id,
            'estimated_completion_time': 30,  # seconds
            'cost': 1.0  # Arbitrary cost metric
        }
        self.broadcast_message('task_bid', bid_message)

    def can_provide_resource(self, resource_type: str, resource_location: Dict) -> bool:
        """
        Check if this robot can provide the requested resource
        """
        # For this example, return True for any resource request
        # In a real implementation, check if we actually have the resource
        return True

    def respond_to_resource_request(self, requester_id: str, resource_type: str):
        """
        Respond to a resource request
        """
        response = {
            'type': 'resource_response',
            'requester_id': requester_id,
            'provider_id': self.robot_id,
            'resource_type': resource_type,
            'availability': True,
            'estimated_wait_time': 10  # seconds
        }
        self.broadcast_message('resource_response', response)

    def execute_command(self, cmd: RobotCommand):
        """
        Execute a received command
        """
        cmd_type = cmd.command_type
        
        if cmd_type == 'navigate_to':
            self.robot_status = 'navigating'
            self.current_task = f'navigate_to_{cmd.command_details}'
            self.get_logger().info(f'Navigating to: {cmd.command_details}')
        elif cmd_type == 'perform_task':
            self.robot_status = 'performing_task'
            self.current_task = cmd.command_details
            self.get_logger().info(f'Performing task: {cmd.command_details}')
        elif cmd_type == 'wait':
            self.robot_status = 'waiting'
            self.get_logger().info('Waiting for further instructions')

    def coordination_loop(self):
        """
        Main coordination loop that runs periodically
        """
        # Clean up old robot states (older than 10 seconds)
        current_time = self.get_clock().now().nanoseconds / 1e9
        expired_robots = [
            robot_id for robot_id, last_time in self.last_update_times.items()
            if current_time - last_time > 10.0
        ]
        
        for robot_id in expired_robots:
            if robot_id in self.other_robots:
                del self.other_robots[robot_id]
            if robot_id in self.last_update_times:
                del self.last_update_times[robot_id]
        
        # Report status of known robots
        self.get_logger().debug(f'Tracking {len(self.other_robots)} other robots')


def main(args=None):
    rclpy.init(args=args)
    communicator = MultiRobotCommunicator()
    
    try:
        rclpy.spin(communicator)
    except KeyboardInterrupt:
        pass
    finally:
        communicator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Task Allocation System

Here's an implementation of a distributed task allocation system:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from multirobot_msgs.msg import Task, TaskAllocation, RobotCapability
import json
import heapq
from typing import Dict, List


class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator')
        
        # Publishers and subscribers
        self.task_allocation_pub = self.create_publisher(TaskAllocation, '/task_allocation', 10)
        self.task_request_sub = self.create_subscription(
            String, '/task_requests', self.task_request_callback, 10)
        self.bid_sub = self.create_subscription(
            String, '/task_bids', self.bid_callback, 10)
        self.robot_state_sub = self.create_subscription(
            String, '/robot_capabilities', self.robot_capabilities_callback, 10)
        
        # Internal state
        self.pending_tasks: List[Task] = []
        self.robot_capabilities: Dict[str, List[str]] = {}
        self.task_bids: Dict[str, List[Dict]] = {}  # task_id -> list of bids
        self.allocated_tasks: Dict[str, str] = {}   # task_id -> robot_id
        
        # Timer for allocation process
        self.allocation_timer = self.create_timer(2.0, self.process_allocations)
        
        self.get_logger().info('Task Allocator initialized')

    def task_request_callback(self, msg: String):
        """
        Callback for receiving task requests
        """
        try:
            task_data = json.loads(msg.data)
            
            # Create a task object
            task = Task()
            task.task_id = task_data.get('task_id', f'task_{len(self.pending_tasks)}')
            task.task_type = task_data.get('task_type', 'unknown')
            task.location_x = task_data.get('location_x', 0.0)
            task.location_y = task_data.get('location_y', 0.0)
            task.priority = task_data.get('priority', 1)
            task.required_capabilities = task_data.get('required_capabilities', [])
            
            # Add to pending tasks
            self.pending_tasks.append(task)
            self.task_bids[task.task_id] = []
            
            self.get_logger().info(f'Received task request: {task.task_id}')
        
        except json.JSONDecodeError:
            self.get_logger().error('Invalid task request JSON')

    def bid_callback(self, msg: String):
        """
        Callback for receiving bids from robots
        """
        try:
            bid_data = json.loads(msg.data)
            
            task_id = bid_data.get('task_id')
            robot_id = bid_data.get('robot_id')
            cost = bid_data.get('cost', float('inf'))
            estimated_time = bid_data.get('estimated_time', float('inf'))
            
            if task_id in self.task_bids:
                bid = {
                    'robot_id': robot_id,
                    'cost': cost,
                    'estimated_time': estimated_time,
                    'timestamp': self.get_clock().now().nanoseconds / 1e9
                }
                self.task_bids[task_id].append(bid)
                
                self.get_logger().debug(f'Received bid for {task_id} from {robot_id}, cost: {cost}')
        
        except json.JSONDecodeError:
            self.get_logger().error('Invalid bid JSON')

    def robot_capabilities_callback(self, msg: String):
        """
        Callback for receiving robot capabilities
        """
        try:
            cap_data = json.loads(msg.data)
            robot_id = cap_data.get('robot_id')
            capabilities = cap_data.get('capabilities', [])
            
            self.robot_capabilities[robot_id] = capabilities
            
        except json.JSONDecodeError:
            self.get_logger().error('Invalid capabilities JSON')

    def process_allocations(self):
        """
        Process task allocations based on received bids
        """
        # For each pending task, allocate to the best bidder
        for task in self.pending_tasks[:]:  # Use slice to avoid modification during iteration
            if task.task_id in self.task_bids and self.task_bids[task.task_id]:
                # Select the robot with the lowest cost bid
                best_bid = min(self.task_bids[task.task_id], key=lambda x: x['cost'])
                
                # Allocate the task
                allocation = TaskAllocation()
                allocation.task_id = task.task_id
                allocation.assigned_robot_id = best_bid['robot_id']
                allocation.estimated_completion_time = best_bid['estimated_time']
                allocation.allocation_time = self.get_clock().now().to_msg()
                
                # Publish the allocation
                self.task_allocation_pub.publish(allocation)
                
                # Update our records
                self.allocated_tasks[task.task_id] = best_bid['robot_id']
                
                # Remove the task from pending list
                self.pending_tasks.remove(task)
                
                self.get_logger().info(f'Allocated task {task.task_id} to robot {best_bid["robot_id"]}')

    def calculate_allocation_score(self, robot_id: str, task: Task) -> float:
        """
        Calculate a score for how suitable a robot is for a task
        Lower score is better
        """
        # Start with base score
        score = 0.0
        
        # Check if robot has required capabilities
        robot_caps = self.robot_capabilities.get(robot_id, [])
        required_caps = task.required_capabilities
        
        missing_caps = [cap for cap in required_caps if cap not in robot_caps]
        if missing_caps:
            return float('inf')  # Robot can't do this task
        
        # Add distance factor (if we know robot location)
        # In a real implementation, we would have the robot's position
        # For this example, we'll just add a base cost
        score += 1.0
        
        # Add complexity factor based on task type
        if task.task_type == 'complex_manipulation':
            score += 5.0
        elif task.task_type == 'simple_navigation':
            score += 1.0
        else:
            score += 2.0
        
        return score


def main(args=None):
    rclpy.init(args=args)
    allocator = TaskAllocator()
    
    try:
        rclpy.spin(allocator)
    except KeyboardInterrupt:
        pass
    finally:
        allocator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Multi-Robot Path Planning with Collision Avoidance

Here's an implementation of multi-robot path planning with collision avoidance:

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from task_allocator import TaskAllocator
import numpy as np
import math
from typing import Dict, List, Tuple


class MultiRobotPathPlanner(Node):
    def __init__(self):
        super().__init__('multirobot_path_planner')
        
        # Publishers
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_viz_pub = self.create_publisher(MarkerArray, '/multirobot_paths', 10)
        self.collision_viz_pub = self.create_publisher(MarkerArray, '/collision_predictions', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.other_paths_sub = self.create_subscription(
            String,
            '/other_robot_paths',
            self.other_path_callback,
            10)
        
        self.task_allocation_sub = self.create_subscription(
            String,
            '/task_allocations',
            self.task_allocation_callback,
            10)
        
        # Internal state
        self.current_pose = None
        self.current_velocity = None
        self.other_robot_paths: Dict[str, List[PoseStamped]] = {}
        self.robot_id = "robot_1"  # In a real system, this would be unique per robot
        self.target_position = None
        self.current_path = []
        
        # Create path planning timer
        self.path_timer = self.create_timer(0.1, self.path_planning_loop)
        
        # Create visualization timer
        self.viz_timer = self.create_timer(0.5, self.visualize_paths)
        
        self.get_logger().info('Multi-Robot Path Planner initialized')

    def odom_callback(self, msg):
        """
        Callback for odometry updates
        """
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def other_path_callback(self, msg: String):
        """
        Callback for receiving paths from other robots
        """
        try:
            path_data = json.loads(msg.data)
            robot_id = path_data.get('robot_id')
            path_points = path_data.get('path', [])
            
            # Convert path points to PoseStamped objects
            pose_path = []
            for point_data in path_points:
                pose = PoseStamped()
                pose.pose.position.x = point_data.get('x', 0)
                pose.pose.position.y = point_data.get('y', 0)
                pose.pose.position.z = point_data.get('z', 0)
                # For simplicity, assume zero orientation
                pose.pose.orientation.w = 1.0
                pose_path.append(pose)
            
            self.other_robot_paths[robot_id] = pose_path
        
        except json.JSONDecodeError:
            self.get_logger().error('Invalid path data JSON')

    def task_allocation_callback(self, msg: String):
        """
        Callback for receiving task allocations
        """
        try:
            allocation_data = json.loads(msg.data)
            target_robot = allocation_data.get('assigned_robot_id')
            
            if target_robot == self.robot_id:
                target_pos_data = allocation_data.get('target_position', {})
                self.target_position = (
                    target_pos_data.get('x', 0.0),
                    target_pos_data.get('y', 0.0),
                    target_pos_data.get('z', 0.0)
                )
                self.get_logger().info(f'Received navigation task to {self.target_position}')
        
        except json.JSONDecodeError:
            self.get_logger().error('Invalid allocation data JSON')

    def path_planning_loop(self):
        """
        Main path planning and execution loop
        """
        if not self.current_pose or not self.target_position:
            return
        
        # Calculate path to target (simplified as straight line)
        current_pos = (
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z
        )
        
        # Check for potential collisions with other robots
        collision_approach_time = self.predict_collisions(current_pos, self.target_position)
        
        # If collision is predicted soon, modify path or speed
        if collision_approach_time < 3.0:  # 3 seconds
            self.get_logger().warn(f'Collision predicted in {collision_approach_time:.1f}s, adjusting path')
            
            # For this example, we'll slow down when collision is predicted
            self.adjust_for_collision(current_pos, collision_approach_time)
        else:
            # Proceed with normal navigation to target
            self.navigate_to_target()

    def predict_collisions(self, current_pos: Tuple[float, float, float], 
                          target_pos: Tuple[float, float, float]) -> float:
        """
        Predict if and when collisions might occur with other robots
        Returns time to closest collision, or float('inf') if no collision predicted
        """
        min_collision_time = float('inf')
        
        if not self.current_velocity:
            return min_collision_time
        
        # For each other robot's predicted path
        for robot_id, path in self.other_robot_paths.items():
            if not path:
                continue
            
            # Predict our position over the next few seconds
            robot_pos = (self.current_pose.position.x, 
                        self.current_pose.position.y, 
                        self.current_pose.position.z)
            robot_vel = (self.current_velocity.linear.x, 
                        self.current_velocity.linear.y, 
                        self.current_velocity.linear.z)
            
            # Sample positions along other robot's path
            for i, pose_stamped in enumerate(path):
                other_pos = (pose_stamped.pose.position.x,
                            pose_stamped.pose.position.y,
                            pose_stamped.pose.position.z)
                
                # Approximate time to reach this point in the path
                time_to_point = i * 0.1  # Assuming path points are 0.1s apart
                
                # Predict our position at that time
                predicted_our_pos = (
                    robot_pos[0] + robot_vel[0] * time_to_point,
                    robot_pos[1] + robot_vel[1] * time_to_point,
                    robot_pos[2] + robot_vel[2] * time_to_point
                )
                
                # Calculate distance between predicted positions
                dist = math.sqrt(
                    (predicted_our_pos[0] - other_pos[0])**2 +
                    (predicted_our_pos[1] - other_pos[1])**2 +
                    (predicted_our_pos[2] - other_pos[2])**2
                )
                
                # If distance is small, predict collision
                collision_distance = 0.5  # meters
                if dist < collision_distance and time_to_point < min_collision_time:
                    min_collision_time = time_to_point
        
        return min_collision_time

    def adjust_for_collision(self, current_pos: Tuple[float, float, float], 
                           collision_time: float):
        """
        Adjust navigation to avoid predicted collision
        """
        # For this example, we'll slow down or stop
        cmd_vel = Twist()
        
        # If collision is very soon, stop completely
        if collision_time < 1.0:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        else:
            # Otherwise, slow down proportionally
            slow_factor = collision_time / 3.0  # Slow down more as collision gets closer
            cmd_vel.linear.x = 0.5 * slow_factor  # Base speed of 0.5 m/s
            cmd_vel.angular.z = 0.0  # No turning for this example
        
        self.velocity_pub.publish(cmd_vel)

    def navigate_to_target(self):
        """
        Navigate to the target position (normal operation)
        """
        if not self.current_pose:
            return
        
        # Calculate direction to target
        target_x, target_y, target_z = self.target_position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        dx = target_x - current_x
        dy = target_y - current_y
        distance_to_target = math.sqrt(dx*dx + dy*dy)
        
        # Calculate required orientation to face target
        required_yaw = math.atan2(dy, dx)
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Normalize angle difference
        angle_diff = required_yaw - current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Create navigation command
        cmd_vel = Twist()
        
        # Move toward target if not too close
        arrival_threshold = 0.5  # meters
        if distance_to_target > arrival_threshold:
            cmd_vel.linear.x = min(0.5, distance_to_target * 0.5)  # Approaching speed
            cmd_vel.angular.z = angle_diff * 1.0  # Rotation to face target
        else:
            # Stop when reached
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        
        self.velocity_pub.publish(cmd_vel)

    def get_yaw_from_quaternion(self, orientation):
        """
        Extract yaw angle from quaternion
        """
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def visualize_paths(self):
        """
        Visualize paths of all robots for debugging
        """
        marker_array = MarkerArray()
        
        # Visualize our current path
        if self.current_path:
            path_marker = Marker()
            path_marker.header.frame_id = 'map'
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = 'current_robot_path'
            path_marker.id = 0
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            
            for pose_stamped in self.current_path:
                path_marker.points.append(pose_stamped.pose.position)
            
            path_marker.scale.x = 0.05  # Line width
            path_marker.color.a = 1.0
            path_marker.color.r = 1.0  # Red for current robot
            path_marker.color.g = 0.0
            path_marker.color.b = 0.0
            
            marker_array.markers.append(path_marker)
        
        # Visualize other robots' paths
        for i, (robot_id, path) in enumerate(self.other_robot_paths.items()):
            if path:
                path_marker = Marker()
                path_marker.header.frame_id = 'map'
                path_marker.header.stamp = self.get_clock().now().to_msg()
                path_marker.ns = f'robot_{robot_id}_path'
                path_marker.id = i + 1
                path_marker.type = Marker.LINE_STRIP
                path_marker.action = Marker.ADD
                
                for pose_stamped in path:
                    path_marker.points.append(pose_stamped.pose.position)
                
                path_marker.scale.x = 0.05  # Line width
                path_marker.color.a = 1.0
                path_marker.color.r = 0.0  # Blue for other robots
                path_marker.color.g = 0.0
                path_marker.color.b = 1.0
                
                marker_array.markers.append(path_marker)
        
        self.path_viz_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    planner = MultiRobotPathPlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercises

1. **Logical Exercise**: Compare centralized vs. decentralized approaches to multi-robot coordination. What are the trade-offs in terms of performance, scalability, and robustness?

2. **Conceptual Exercise**: Explain the concept of conflict resolution in multi-robot systems. How can robots resolve resource conflicts or path conflicts without a central authority?

3. **Implementation Exercise**: Create a complete multi-robot system with two robots that must coordinate to transport a large object that requires both robots working together. Implement task allocation, path planning, and collision avoidance.

## Summary

This chapter covered multi-robot systems and coordination, including communication protocols, task allocation, and path planning for multiple robots. We explored the challenges of coordinating multiple agents and implemented examples of multi-robot communication and task allocation.

Multi-robot systems provide significant advantages in terms of redundancy, parallelism, and scalability. However, they also introduce challenges in coordination, communication, and conflict resolution. Understanding these concepts is essential for developing effective multi-robot applications.

## References
- [Multi-Robot Systems: From Swarms to Intelligent Automata](https://link.springer.com/book/10.1007/978-94-017-0373-0)
- [Distributed Robotics: State-of-the-Art and Future Directions](https://ieeexplore.ieee.org/document/6343147)
- [Multi-Agent Path Finding for Large-Scale Settings](https://www.sciencedirect.com/science/article/pii/S0004370220301234)