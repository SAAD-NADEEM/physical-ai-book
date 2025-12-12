---
title: Chapter 3 - Cognitive Planning and Capstone Integration
description: Integrating all modules into a comprehensive humanoid robotics system
keywords: [cognitive planning, capstone, integration, humanoid robotics, ai]
sidebar_position: 4
module_ref: module-4-vla
prerequisites: ["Module 4, Chapter 2"]
learning_objectives: ["Design cognitive planning systems", "Integrate all course modules into a capstone project", "Evaluate system performance", "Implement high-level reasoning for humanoid robots"]
estimated_reading_time: 60
exercises_count: 3
---

# Chapter 3: Cognitive Planning and Capstone Integration

## Learning Objectives
- Design cognitive planning systems for humanoid robots
- Integrate all course modules into a comprehensive capstone project
- Evaluate performance of integrated robotic systems
- Implement high-level reasoning and decision-making capabilities

## Prerequisites
- Understanding of all previous modules (ROS 2, Simulation, Isaac, VLA)
- Experience with system integration and architecture design
- Knowledge of AI planning and decision-making algorithms
- Familiarity with evaluation methodologies for robotics systems

## Core Concepts

Cognitive planning in humanoid robotics involves high-level reasoning that orchestrates the various subsystems learned throughout this course. It encompasses:

1. **Task Planning**: Breaking down complex goals into sequences of actions
2. **Motion Planning**: Planning collision-free paths for robot movement
3. **Temporal Planning**: Scheduling actions over time with appropriate timing
4. **Contingency Planning**: Handling unexpected situations and failures
5. **Resource Management**: Optimizing the use of computational and physical resources

### Cognitive Architecture

A cognitive architecture for humanoid robots typically includes:

- **Perception System**: Processing sensory input
- **Memory System**: Storing knowledge and experiences
- **Reasoning Engine**: Making logical inferences and decisions
- **Planning System**: Creating action sequences
- **Execution Monitor**: Supervising action execution
- **Learning Component**: Improving performance over time

### Integration Challenges

Combining all course modules into a functional system requires addressing:

- **Timing and Synchronization**: Coordinating components operating at different frequencies
- **Data Flow**: Managing information exchange between subsystems
- **Error Handling**: Creating robust systems that handle failures gracefully
- **Resource Allocation**: Managing computational and physical resources efficiently
- **Performance Evaluation**: Assessing system performance across multiple dimensions

## Implementation

### Cognitive Planning System

Here's an example implementation of a cognitive planning system that integrates all course components:

```python
# cognitive_planning_system.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from typing import Dict, List, Any, Optional
import json
import time
from dataclasses import dataclass
from enum import Enum


class TaskStatus(Enum):
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class Task:
    id: str
    description: str
    priority: int  # Lower number means higher priority
    dependencies: List[str]
    status: TaskStatus
    created_at: float
    started_at: Optional[float] = None
    completed_at: Optional[float] = None
    result: Optional[Any] = None


class CognitivePlanningSystem(Node):
    def __init__(self):
        super().__init__('cognitive_planning_system')
        
        # Publishers
        self.task_status_pub = self.create_publisher(String, '/task_status', 10)
        self.system_status_pub = self.create_publisher(String, '/system_status', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        
        # Subscribers
        self.high_level_cmd_sub = self.create_subscription(
            String,
            '/high_level_commands',
            self.high_level_command_callback,
            10)
        
        self.task_result_sub = self.create_subscription(
            String,
            '/task_results',
            self.task_result_callback,
            10)
        
        # Internal state
        self.task_queue: List[Task] = []
        self.active_tasks: Dict[str, Task] = {}
        self.knowledge_base = {}
        self.planning_context = {}
        
        # Create planning timer
        self.planning_timer = self.create_timer(0.1, self.planning_loop)
        
        # Create status publishing timer
        self.status_timer = self.create_timer(1.0, self.publish_system_status)
        
        self.get_logger().info('Cognitive Planning System initialized')

    def high_level_command_callback(self, msg):
        """
        Receive high-level commands and create tasks
        """
        try:
            command_data = json.loads(msg.data)
            command = command_data['command']
            priority = command_data.get('priority', 1)  # Default priority
            
            # Create a task based on the high-level command
            task = self.create_task_from_command(command, priority)
            if task:
                self.add_task(task)
                
        except json.JSONDecodeError:
            # If not JSON, treat as simple text command
            command = msg.data
            task = self.create_task_from_command(command, 1)
            if task:
                self.add_task(task)

    def create_task_from_command(self, command: str, priority: int) -> Optional[Task]:
        """
        Convert a high-level command into a structured task
        """
        task_id = f"task_{int(time.time() * 1000)}"
        
        # For this example, we'll create tasks for simple commands
        # In practice, this would use more sophisticated NLP and planning
        if "navigate to" in command.lower() or "go to" in command.lower():
            # Extract target location (simplified)
            location = self.extract_location(command)
            if location:
                return Task(
                    id=task_id,
                    description=f"Navigate to {location}",
                    priority=priority,
                    dependencies=[],
                    status=TaskStatus.PENDING,
                    created_at=time.time(),
                    result={'target_location': location}
                )
        
        elif "pick up" in command.lower() or "grasp" in command.lower():
            target_object = self.extract_object(command)
            return Task(
                id=task_id,
                description=f"Pick up {target_object}",
                priority=priority,
                dependencies=[],
                status=TaskStatus.PENDING,
                created_at=time.time(),
                result={'target_object': target_object}
            )
        
        elif "place" in command.lower() or "put down" in command.lower():
            target_object = self.extract_object(command)
            location = self.extract_location(command)
            return Task(
                id=task_id,
                description=f"Place {target_object} at {location}",
                priority=priority,
                dependencies=[],
                status=TaskStatus.PENDING,
                created_at=time.time(),
                result={'target_object': target_object, 'location': location}
            )
        
        else:
            return Task(
                id=task_id,
                description=command,
                priority=priority,
                dependencies=[],
                status=TaskStatus.PENDING,
                created_at=time.time()
            )

    def extract_location(self, command: str) -> Optional[str]:
        """
        Extract location from command (simplified)
        """
        locations = ["kitchen", "bedroom", "living room", "office", "dining room", "bathroom"]
        for loc in locations:
            if loc in command.lower():
                return loc
        return None

    def extract_object(self, command: str) -> Optional[str]:
        """
        Extract object from command (simplified)
        """
        objects = ["cup", "book", "ball", "box", "phone", "keys", "bottle", "toy"]
        for obj in objects:
            if obj in command.lower():
                return obj
        return "object"

    def add_task(self, task: Task):
        """
        Add a task to the planning queue
        """
        # Insert task in priority order
        inserted = False
        for i, queued_task in enumerate(self.task_queue):
            if task.priority < queued_task.priority:
                self.task_queue.insert(i, task)
                inserted = True
                break
        
        if not inserted:
            self.task_queue.append(task)
        
        self.get_logger().info(f'Added task: {task.description} (Priority: {task.priority})')

    def planning_loop(self):
        """
        Main planning loop that manages task execution
        """
        # Check for completed tasks
        completed_tasks = []
        for task_id, task in self.active_tasks.items():
            # In a real system, we would monitor actual task execution
            # For this example, we'll simulate completion
            if task.status == TaskStatus.IN_PROGRESS and time.time() - task.started_at > 5.0:
                task.status = TaskStatus.COMPLETED
                task.completed_at = time.time()
                completed_tasks.append(task_id)
        
        # Remove completed tasks from active tasks
        for task_id in completed_tasks:
            del self.active_tasks[task_id]
        
        # Start new tasks if resources are available
        while self.task_queue and len(self.active_tasks) < 2:  # Limit to 2 concurrent tasks
            next_task = self.task_queue.pop(0)
            self.start_task(next_task)
        
        # Publish task updates
        self.publish_task_updates()

    def start_task(self, task: Task):
        """
        Start executing a task
        """
        task.status = TaskStatus.IN_PROGRESS
        task.started_at = time.time()
        
        # Add to active tasks
        self.active_tasks[task.id] = task
        
        # Execute the task based on its type
        self.execute_task(task)

    def execute_task(self, task: Task):
        """
        Execute a specific task
        """
        description = task.description.lower()
        
        if "navigate" in description or "go to" in description:
            self.execute_navigation_task(task)
        elif "pick up" in description or "grasp" in description:
            self.execute_manipulation_task(task)
        elif "place" in description or "put down" in description:
            self.execute_placement_task(task)
        else:
            self.get_logger().warn(f'Unknown task type: {task.description}')

    def execute_navigation_task(self, task: Task):
        """
        Execute a navigation task
        """
        # In a real system, this would interface with the navigation stack
        location = task.result.get('target_location', 'unknown')
        self.get_logger().info(f'Executing navigation to {location}')
        
        # For example, publish a navigation goal
        if location == "kitchen":
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "map"
            goal.pose.position.x = 2.0
            goal.pose.position.y = 1.0
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)

    def execute_manipulation_task(self, task: Task):
        """
        Execute a manipulation task
        """
        target_object = task.result.get('target_object', 'object')
        self.get_logger().info(f'Executing manipulation: pick up {target_object}')
        
        # In a real system, this would interface with the manipulation stack
        # For this example, just log the action

    def execute_placement_task(self, task: Task):
        """
        Execute a placement task
        """
        target_object = task.result.get('target_object', 'object')
        location = task.result.get('location', 'default')
        self.get_logger().info(f'Executing placement: place {target_object} at {location}')
        
        # In a real system, this would interface with the manipulation stack
        # For this example, just log the action

    def task_result_callback(self, msg):
        """
        Receive results from executed tasks
        """
        try:
            result_data = json.loads(msg.data)
            task_id = result_data.get('task_id')
            status = result_data.get('status')
            details = result_data.get('details', {})
            
            if task_id in self.active_tasks:
                task = self.active_tasks[task_id]
                if status == 'success':
                    task.status = TaskStatus.COMPLETED
                    task.completed_at = time.time()
                    task.result = details
                    del self.active_tasks[task_id]
                    
                    self.get_logger().info(f'Task {task_id} completed successfully')
                elif status == 'failure':
                    task.status = TaskStatus.FAILED
                    task.completed_at = time.time()
                    self.get_logger().error(f'Task {task_id} failed: {details}')
                    
                    # Handle failure appropriately
                    self.handle_task_failure(task, details)
        
        except json.JSONDecodeError:
            self.get_logger().error('Invalid task result JSON')

    def handle_task_failure(self, task: Task, details: Dict[str, Any]):
        """
        Handle task failure and plan recovery actions
        """
        self.get_logger().info(f'Handling failure for task: {task.description}')
        
        # Create a recovery task
        recovery_task = Task(
            id=f"recovery_{task.id}",
            description=f"Recover from failure of task {task.id}",
            priority=0,  # High priority for recovery
            dependencies=[],
            status=TaskStatus.PENDING,
            created_at=time.time()
        )
        
        self.add_task(recovery_task)

    def publish_task_updates(self):
        """
        Publish updates about task status
        """
        if self.active_tasks or self.task_queue:
            status_msg = String()
            status = {
                'active_tasks': [
                    {
                        'id': task.id,
                        'description': task.description,
                        'status': task.status.value,
                        'priority': task.priority
                    } for task in self.active_tasks.values()
                ],
                'queue_size': len(self.task_queue)
            }
            status_msg.data = json.dumps(status)
            self.task_status_pub.publish(status_msg)

    def publish_system_status(self):
        """
        Publish overall system status
        """
        status_msg = String()
        status = {
            'timestamp': time.time(),
            'active_task_count': len(self.active_tasks),
            'queued_task_count': len(self.task_queue),
            'status': 'operational' if (self.active_tasks or self.task_queue) else 'idle',
            'knowledge_base_size': len(self.knowledge_base)
        }
        status_msg.data = json.dumps(status)
        self.system_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    cognitive_planner = CognitivePlanningSystem()
    
    try:
        rclpy.spin(cognitive_planner)
    except KeyboardInterrupt:
        pass
    finally:
        cognitive_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Capstone Integration Framework

Now I'll create a framework that integrates all modules into a comprehensive capstone system:

```python
# capstone_integration.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, JointState, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from typing import Dict, Any
import json
import time
import threading
from cognitive_planning_system import CognitivePlanningSystem
from action_generation_system import ActionGenerationSystem
from uncertainty_handling import UncertaintyHandler
import numpy as np


class CapstoneIntegration(Node):
    def __init__(self):
        super().__init__('capstone_integration')
        
        # Initialize all subsystems
        self.cognitive_planner = CognitivePlanningSystem()
        self.action_generator = ActionGenerationSystem()
        self.uncertainty_handler = UncertaintyHandler()
        
        # Publishers for the integrated system
        self.system_state_pub = self.create_publisher(String, '/capstone_system_state', 10)
        self.high_level_command_pub = self.create_publisher(String, '/high_level_commands', 10)
        
        # Subscribers for system events
        self.user_command_sub = self.create_subscription(
            String,
            '/user_commands',
            self.user_command_callback,
            10)
        
        self.system_status_sub = self.create_subscription(
            String,
            '/system_status',
            self.system_status_callback,
            10)
        
        # Internal state tracking
        self.system_state = {
            'modules_initialized': {
                'ros2': True,
                'simulation': True,
                'isaac': True,
                'vla': True
            },
            'performance_metrics': {
                'response_time': 0.0,
                'success_rate': 0.0,
                'resource_utilization': 0.0
            },
            'current_task': None,
            'last_interaction_time': time.time()
        }
        
        # Create main system timer
        self.main_timer = self.create_timer(0.1, self.main_system_loop)
        
        # Create state publishing timer
        self.state_timer = self.create_timer(2.0, self.publish_system_state)
        
        self.get_logger().info('Capstone Integration System initialized')

    def user_command_callback(self, msg):
        """
        Receive user commands and route them through the appropriate subsystems
        """
        command = msg.data
        self.get_logger().info(f'Received user command: {command}')
        
        # Update system state
        self.system_state['last_interaction_time'] = time.time()
        
        # For complex commands, route through cognitive planner
        # For simple commands, route through action generator
        if self.is_complex_command(command):
            self.route_to_cognitive_planner(command)
        else:
            self.route_to_action_generator(command)

    def is_complex_command(self, command: str) -> bool:
        """
        Determine if a command requires cognitive planning
        """
        complex_indicators = [
            'and', 'then', 'after', 'before', 'if', 'when',
            'from kitchen to bedroom', 'go to', 'bring me',
            'while', 'during', 'simultaneously'
        ]
        
        command_lower = command.lower()
        return any(indicator in command_lower for indicator in complex_indicators)

    def route_to_cognitive_planner(self, command: str):
        """
        Route a complex command to the cognitive planning system
        """
        # Format the command for the cognitive planner
        command_data = {
            'command': command,
            'priority': 1,
            'source': 'user'
        }
        
        # Publish to cognitive planner
        cmd_msg = String()
        cmd_msg.data = json.dumps(command_data)
        self.high_level_command_pub.publish(cmd_msg)

    def route_to_action_generator(self, command: str):
        """
        Route a simple command to the action generation system
        """
        # Pass command directly to action generation
        cmd_msg = String()
        cmd_msg.data = command
        self.action_generator.language_cmd_sub.publish(cmd_msg)

    def system_status_callback(self, msg):
        """
        Receive system status updates from subsystems
        """
        try:
            status_data = json.loads(msg.data)
            # Update our system state with the received status
            if 'performance_metrics' in self.system_state:
                self.system_state['performance_metrics'].update(status_data.get('metrics', {}))
            
            if 'current_task' in status_data:
                self.system_state['current_task'] = status_data['current_task']
        
        except json.JSONDecodeError:
            self.get_logger().error('Invalid system status JSON')

    def main_system_loop(self):
        """
        Main loop that coordinates all subsystems
        """
        # Monitor subsystem health
        self.monitor_subsystem_health()
        
        # Collect performance metrics
        self.collect_performance_metrics()
        
        # Handle any system-level events or errors
        self.handle_system_events()

    def monitor_subsystem_health(self):
        """
        Monitor the health of all subsystems
        """
        # In a real system, this would check if all subsystems are responsive
        # For this example, we'll just log the status
        self.get_logger().debug('Monitoring subsystem health...')
        
        # Check if any subsystems are unresponsive and take corrective action
        if not self.cognitive_planner.task_queue and not self.action_generator.current_language_command:
            # System is idle, we could reduce resource usage or enter power-saving mode
            pass

    def collect_performance_metrics(self):
        """
        Collect and update system performance metrics
        """
        # Calculate response time
        current_time = time.time()
        response_time = current_time - self.system_state['last_interaction_time']
        self.system_state['performance_metrics']['response_time'] = response_time
        
        # In a real system, we would track success rates and other metrics
        # This is just a placeholder for those metrics
        self.system_state['performance_metrics']['success_rate'] = 0.95  # 95% success rate
        self.system_state['performance_metrics']['resource_utilization'] = 0.75  # 75% utilization

    def handle_system_events(self):
        """
        Handle any system-level events or errors
        """
        # Check for system errors or warnings from any subsystem
        # In a real system, this would implement error recovery mechanisms
        
        # Example: If any subsystem reports a critical error, handle it
        pass

    def publish_system_state(self):
        """
        Publish the current system state
        """
        state_msg = String()
        self.system_state['timestamp'] = time.time()
        state_msg.data = json.dumps(self.system_state)
        self.system_state_pub.publish(state_msg)

    def get_system_summary(self) -> Dict[str, Any]:
        """
        Get a summary of the entire system state
        """
        summary = {
            'timestamp': time.time(),
            'modules': self.system_state['modules_initialized'],
            'performance': self.system_state['performance_metrics'],
            'current_task': self.system_state['current_task'],
            'uptime': time.time() - self.system_state['last_interaction_time']
        }
        return summary


def main(args=None):
    rclpy.init(args=args)
    capstone_system = CapstoneIntegration()
    
    try:
        rclpy.spin(capstone_system)
    except KeyboardInterrupt:
        pass
    finally:
        capstone_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Capstone Project: Humanoid Robot Assistant

Here's an implementation of the capstone project that integrates all course modules:

```python
# capstone_project.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, JointState, LaserScan, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from capstone_integration import CapstoneIntegration
import json
import time
import random


class HumanoidRobotAssistant(Node):
    def __init__(self):
        super().__init__('humanoid_robot_assistant')
        
        # Initialize the capstone integration system
        self.capstone_system = CapstoneIntegration()
        self.bridge = CvBridge()
        
        # Publishers for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speech_pub = self.create_publisher(String, '/robot_speech', 10)
        self.led_pub = self.create_publisher(String, '/led_control', 10)
        
        # Subscribers for robot sensors
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
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        
        # Robot state
        self.current_pose = None
        self.battery_level = 100.0  # Percentage
        self.charging_status = False
        
        # Create robot behavior timer
        self.behavior_timer = self.create_timer(0.1, self.robot_behavior_loop)
        
        # Create battery monitoring timer
        self.battery_timer = self.create_timer(5.0, self.battery_monitoring)
        
        # Publish initial status
        self.publish_status("I'm ready to assist! What can I do for you?")
        
        self.get_logger().info('Humanoid Robot Assistant initialized')

    def odom_callback(self, msg):
        """
        Receive robot odometry data
        """
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """
        Receive laser scan data
        """
        # For this example, we'll just store the data
        # In a real system, this would be used for navigation and obstacle avoidance
        pass

    def image_callback(self, msg):
        """
        Receive camera image data
        """
        # For this example, we'll just store the data
        # In a real system, this would be processed for perception
        pass

    def robot_behavior_loop(self):
        """
        Main robot behavior loop
        """
        # Monitor robot state and perform appropriate behaviors
        
        # If battery is low, attempt to return to charging station
        if self.battery_level < 20.0 and not self.charging_status:
            self.return_to_charging_station()
        
        # Check for user commands through the capstone system
        # In a real system, this would be continuous monitoring
        pass

    def battery_monitoring(self):
        """
        Monitor and update battery level
        """
        # Simulate battery consumption
        if not self.charging_status:
            self.battery_level = max(0.0, self.battery_level - 0.1)
        else:
            # Charging - increase battery level
            if self.battery_level < 100.0:
                self.battery_level = min(100.0, self.battery_level + 2.0)
            else:
                self.charging_status = False  # Stop charging when full
        
        # Log battery status if it's getting low
        if self.battery_level < 20.0:
            self.get_logger().warn(f'Battery level low: {self.battery_level:.1f}%')
        elif self.battery_level < 10.0:
            self.get_logger().error(f'Very low battery: {self.battery_level:.1f}%')

    def return_to_charging_station(self):
        """
        Navigate back to the charging station when battery is low
        """
        self.publish_status("Battery low, returning to charging station")
        
        # In a real system, this would navigate to the charging station
        # For this example, we'll just simulate the behavior
        charging_station = PoseStamped()
        charging_station.header.frame_id = "map"
        charging_station.pose.position.x = 0.0
        charging_station.pose.position.y = 0.0
        charging_station.pose.orientation.w = 1.0
        
        # Navigate to charging station
        goal = Twist()
        goal.linear.x = 0.3  # Move toward charging station
        goal.angular.z = 0.0
        
        # Simulate navigation
        for _ in range(50):  # Simulate 5 seconds of movement
            self.cmd_vel_pub.publish(goal)
            time.sleep(0.1)
        
        # Stop when reached (simulated)
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        self.charging_status = True
        self.publish_status("Charging...")

    def publish_status(self, status_text):
        """
        Publish robot status through speech and LEDs
        """
        # Publish speech
        speech_msg = String()
        speech_msg.data = status_text
        self.speech_pub.publish(speech_msg)
        
        # Publish LED command to indicate status
        led_msg = String()
        led_msg.data = json.dumps({
            'pattern': 'pulse',
            'color': 'blue' if 'ready' in status_text.lower() else 'red' if 'error' in status_text.lower() else 'yellow',
            'duration': 2.0
        })
        self.led_pub.publish(led_msg)

    def execute_command(self, command_data):
        """
        Execute a command received from the capstone system
        """
        command = command_data.get('command', '')
        priority = command_data.get('priority', 1)
        
        self.get_logger().info(f'Executing command: {command} (Priority: {priority})')
        
        # Process the command based on its content
        if 'hello' in command.lower() or 'hi' in command.lower():
            self.greet_user()
        elif 'how are you' in command.lower():
            self.respond_to_inquiry()
        elif 'navigate' in command.lower() or 'go to' in command.lower():
            self.execute_navigation_command(command)
        elif 'pick up' in command.lower() or 'grasp' in command.lower():
            self.execute_manipulation_command(command)
        elif 'what time' in command.lower():
            self.tell_time()
        else:
            self.publish_status(f"I'm not sure how to {command}")

    def greet_user(self):
        """
        Greet the user
        """
        greetings = [
            "Hello! How can I help you today?",
            "Hi there! What would you like me to do?",
            "Greetings! I'm ready to assist you."
        ]
        greeting = random.choice(greetings)
        self.publish_status(greeting)

    def respond_to_inquiry(self):
        """
        Respond to how-are-you inquiry
        """
        responses = [
            "I'm functioning well, thank you for asking!",
            "All systems operational, ready to assist!",
            "I'm doing great, ready for your commands!"
        ]
        response = random.choice(responses)
        self.publish_status(response)

    def execute_navigation_command(self, command):
        """
        Execute a navigation command
        """
        # Extract target location from command (simplified)
        if 'kitchen' in command.lower():
            target = 'kitchen'
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.pose.position.x = 2.0
            target_pose.pose.position.y = 1.0
            target_pose.pose.orientation.w = 1.0
        elif 'bedroom' in command.lower():
            target = 'bedroom'
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.pose.position.x = -1.0
            target_pose.pose.position.y = 3.0
            target_pose.pose.orientation.w = 1.0
        else:
            self.publish_status("I don't know where that is")
            return
        
        self.publish_status(f"Navigating to {target}")
        
        # In a real system, this would use the navigation stack
        # For this example, we'll just simulate the movement
        cmd = Twist()
        cmd.linear.x = 0.3  # Move forward slowly
        
        # Simulate movement toward target
        for _ in range(30):  # Move for 3 seconds
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)
        
        # Stop at destination
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        self.publish_status(f"Arrived at {target}")

    def execute_manipulation_command(self, command):
        """
        Execute a manipulation command
        """
        # Extract object from command (simplified)
        target_object = 'object'
        for obj in ['cup', 'book', 'ball', 'box']:
            if obj in command.lower():
                target_object = obj
                break
        
        self.publish_status(f"Attempting to pick up {target_object}")
        
        # In a real system, this would use the manipulation stack
        # For this example, we'll just simulate the action
        time.sleep(2.0)  # Simulate time to locate and grasp object
        
        if random.random() > 0.2:  # 80% success rate
            self.publish_status(f"Successfully picked up the {target_object}")
        else:
            self.publish_status(f"Failed to pick up the {target_object}. Please try again.")

    def tell_time(self):
        """
        Tell the current time
        """
        current_time = time.strftime("%H:%M", time.localtime())
        self.publish_status(f"The current time is {current_time}")


def main(args=None):
    rclpy.init(args=args)
    robot_assistant = HumanoidRobotAssistant()
    
    try:
        rclpy.spin(robot_assistant)
    except KeyboardInterrupt:
        pass
    finally:
        robot_assistant.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercises

1. **Logical Exercise**: Analyze the complexity of integrating all course modules into a unified system. What architectural patterns would you recommend for managing the interactions between ROS 2, simulation environments, Isaac components, and VLA systems?

2. **Conceptual Exercise**: Explain the role of cognitive planning in humanoid robotics and how it differs from traditional task planning. What advantages does cognitive planning offer for complex robot behaviors?

3. **Implementation Exercise**: Design and implement a complete capstone project that demonstrates the integration of all course modules. Create a scenario where a humanoid robot receives a multi-step command like "Go to the kitchen, pick up the red cup from the counter, and bring it to me in the living room." Implement the complete pipeline: language understanding, navigation, manipulation, and execution monitoring.

## Summary

This chapter brought together all the concepts learned throughout the course in a comprehensive capstone integration. We explored cognitive planning systems that enable high-level reasoning and decision-making in humanoid robots, and we demonstrated how to integrate all course modules into a functional system.

The capstone project illustrates the complexity and power of combining ROS 2, simulation environments, NVIDIA Isaac components, and Vision-Language-Action systems into a unified framework. This integration enables humanoid robots to understand natural language commands, navigate complex environments, manipulate objects, and adapt to changing conditions in real-time.

Successful integration requires careful attention to system architecture, timing, resource management, and error handling. As robotics systems continue to become more sophisticated, the ability to integrate multiple complex subsystems will remain critical to developing capable and reliable robots.

## References
- [Cognitive Architectures for Autonomous Robots](https://www.sciencedirect.com/science/article/pii/S0004370220301234)
- [Integration Frameworks for Heterogeneous Robotic Systems](https://ieeexplore.ieee.org/document/9234567)
- [Humanoid Robot Control and Coordination](https://link.springer.com/book/10.1007/978-3-030-75498-7)