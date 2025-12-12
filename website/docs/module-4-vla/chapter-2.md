---
title: Chapter 2 - Action Generation and Execution
description: Generating robot actions based on vision-language inputs and executing complex tasks
keywords: [action generation, robotics, vision-language-action, task execution]
sidebar_position: 3
module_ref: module-4-vla
prerequisites: ["Module 4, Chapter 1"]
learning_objectives: ["Generate robot actions from vision-language inputs", "Plan and execute complex actions", "Integrate perception, language, and action systems", "Handle uncertainty in action execution"]
estimated_reading_time: 60
exercises_count: 3
---

# Chapter 2: Action Generation and Execution

## Learning Objectives
- Generate robot actions based on vision-language inputs
- Plan and execute complex actions for humanoid robots
- Integrate perception, language, and action systems
- Handle uncertainty and adapt during action execution

## Prerequisites
- Understanding of vision-language models from Chapter 1
- Knowledge of robot control and kinematics
- Experience with task planning concepts
- Familiarity with ROS 2 action servers and clients

## Core Concepts

Action generation in Vision-Language-Action (VLA) systems bridges high-level language commands with low-level robot control. This process involves translating natural language instructions into executable robot behaviors while considering the current state of the environment and the robot.

### Action Generation Pipeline

The action generation process typically involves several stages:

1. **Command Interpretation**: Understanding the natural language command using VLMs
2. **Environment Perception**: Understanding the current state of the environment
3. **Task Planning**: Breaking down the high-level task into a sequence of actions
4. **Action Selection**: Choosing appropriate primitive actions for the task
5. **Execution Monitoring**: Supervising action execution and handling exceptions
6. **Adaptation**: Adjusting the plan based on execution feedback

### Types of Actions

Robotic actions can be categorized as:

1. **Navigation Actions**: Moving the robot to specific locations
2. **Manipulation Actions**: Interacting with objects in the environment
3. **Perception Actions**: Gathering information about the environment
4. **Communication Actions**: Providing feedback to humans
5. **Composite Actions**: Sequences of simpler actions

### Uncertainty Handling

Robots operating in real environments must handle various uncertainties:

- **Perception Uncertainty**: Imperfect sensing of the environment
- **Action Uncertainty**: Variability in action execution
- **Model Uncertainty**: Inaccuracies in robot and environment models
- **Temporal Uncertainty**: Changes in the environment over time

## Implementation

### Action Generation System

Here's an example implementation of a system that generates actions from vision-language inputs:

```python
# action_generation_system.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Pose, Point, Twist
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
from rclpy.action import ActionClient
import openai  # For language understanding
import numpy as np
from collections import deque
import time


class ActionGenerationSystem(Node):
    def __init__(self):
        super().__init__('action_generation_system')
        
        # Publishers for different types of actions
        self.nav_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.manip_cmd_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Subscribers
        self.language_cmd_sub = self.create_subscription(
            String,
            '/natural_language_commands',
            self.language_command_callback,
            10)
        
        self.vision_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.vision_callback,
            10)
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Action clients
        self.move_group_client = ActionClient(self, MoveGroupAction, '/move_group')
        
        # Internal state
        self.current_language_command = None
        self.current_vision_data = None
        self.current_joint_states = None
        self.command_history = deque(maxlen=10)
        
        # Robot capabilities
        self.robot_capabilities = {
            'navigation': True,
            'manipulation': True,
            'grasping': True,
            'vision': True
        }
        
        # Create processing timer
        self.process_timer = self.create_timer(0.1, self.process_commands)
        
        self.get_logger().info('Action Generation System initialized')

    def language_command_callback(self, msg):
        """
        Receive and store natural language commands
        """
        self.current_language_command = msg.data
        self.command_history.append({
            'timestamp': time.time(),
            'command': msg.data,
            'status': 'received'
        })
        self.get_logger().info(f'Received language command: {msg.data}')

    def vision_callback(self, msg):
        """
        Receive and store vision data
        """
        self.current_vision_data = msg

    def joint_state_callback(self, msg):
        """
        Receive and store joint state data
        """
        self.current_joint_states = msg

    def process_commands(self):
        """
        Main processing loop for generating and executing actions
        """
        if not self.current_language_command:
            return
            
        # Parse the command using language understanding
        parsed_command = self.parse_language_command(self.current_language_command)
        
        if not parsed_command:
            self.get_logger().error('Failed to parse language command')
            return
            
        # Plan the task based on the parsed command and current state
        action_plan = self.plan_task(parsed_command)
        
        if not action_plan:
            self.get_logger().error('Failed to generate action plan')
            return
            
        # Execute the action plan
        success = self.execute_action_plan(action_plan)
        
        # Update command status
        for cmd in reversed(self.command_history):
            if cmd['command'] == self.current_language_command:
                cmd['status'] = 'completed' if success else 'failed'
                break
        
        # Clear the current command if successfully processed
        if success:
            self.current_language_command = None

    def parse_language_command(self, command):
        """
        Parse natural language command into structured action
        In practice, this would use large language models or NLP techniques
        """
        # Simplified parsing - in practice, use LLM with structured output
        command_lower = command.lower()
        
        # Define action patterns
        if 'navigate to' in command_lower or 'go to' in command_lower:
            # Extract target location
            target = self.extract_location(command_lower)
            return {
                'action_type': 'navigation',
                'target_location': target,
                'description': command
            }
        elif 'pick up' in command_lower or 'grasp' in command_lower:
            # Extract target object
            target_object = self.extract_object(command_lower)
            return {
                'action_type': 'manipulation',
                'task': 'pick_up',
                'target_object': target_object,
                'description': command
            }
        elif 'place' in command_lower or 'put down' in command_lower:
            # Extract target object and location
            target_object = self.extract_object(command_lower)
            target_location = self.extract_location(command_lower)
            return {
                'action_type': 'manipulation',
                'task': 'place',
                'target_object': target_object,
                'target_location': target_location,
                'description': command
            }
        elif 'move' in command_lower and ('arm' in command_lower or 'hand' in command_lower):
            # Extract pose or position
            target_pose = self.extract_pose(command_lower)
            return {
                'action_type': 'manipulation',
                'task': 'move_arm',
                'target_pose': target_pose,
                'description': command
            }
        else:
            # For other commands, classify them as high-level tasks
            return {
                'action_type': 'high_level',
                'command': command,
                'description': 'Unstructured command - requires further processing'
            }

    def extract_location(self, command):
        """
        Extract location from command (simplified approach)
        In practice, use more sophisticated NLP or vision-based localization
        """
        # Predefined locations in the environment
        locations = {
            'kitchen': {'x': 2.0, 'y': 1.0},
            'bedroom': {'x': -1.0, 'y': 3.0},
            'living room': {'x': 0.0, 'y': 0.0},
            'office': {'x': 3.0, 'y': -1.0},
            'table': {'x': 1.0, 'y': 0.5},  # Near the table
            'couch': {'x': -0.5, 'y': 1.5},  # Near the couch
        }
        
        for loc_name, loc_coords in locations.items():
            if loc_name in command:
                return loc_coords
        
        # If no predefined location found, return None
        # The system would need to use other methods to determine location
        return None

    def extract_object(self, command):
        """
        Extract object from command (simplified approach)
        """
        objects = [
            'red cup', 'blue ball', 'green book', 'white box',
            'cup', 'ball', 'book', 'box', 'bottle', 'phone'
        ]
        
        for obj in objects:
            if obj in command:
                return obj
        
        # If no specific object, return generic
        return 'object'

    def extract_pose(self, command):
        """
        Extract pose or position from command (simplified approach)
        """
        # This is a simplified extraction
        # In practice, use more sophisticated methods
        if 'up' in command:
            return {'position': 'up', 'relative': True}
        elif 'down' in command:
            return {'position': 'down', 'relative': True}
        elif 'forward' in command:
            return {'position': 'forward', 'relative': True}
        else:
            return {'position': 'default', 'relative': False}

    def plan_task(self, parsed_command):
        """
        Plan the sequence of actions to execute the parsed command
        """
        action_type = parsed_command['action_type']
        
        if action_type == 'navigation':
            return self.plan_navigation_task(parsed_command)
        elif action_type == 'manipulation':
            return self.plan_manipulation_task(parsed_command)
        elif action_type == 'high_level':
            # For high-level commands, delegate to higher-level planner
            return self.plan_high_level_task(parsed_command)
        else:
            return None

    def plan_navigation_task(self, parsed_command):
        """
        Plan navigation task
        """
        target_location = parsed_command['target_location']
        
        if not target_location:
            self.get_logger().error('No target location specified for navigation')
            return None
            
        # Create navigation plan
        plan = {
            'task_type': 'navigation',
            'steps': [
                {
                    'action': 'move_to',
                    'target': target_location,
                    'type': 'navigation'
                }
            ]
        }
        
        return plan

    def plan_manipulation_task(self, parsed_command):
        """
        Plan manipulation task
        """
        task = parsed_command['task']
        
        if task == 'pick_up':
            target_object = parsed_command['target_object']
            plan = {
                'task_type': 'manipulation',
                'steps': [
                    {
                        'action': 'approach_object',
                        'target': target_object,
                        'type': 'navigation'
                    },
                    {
                        'action': 'locate_object',
                        'target': target_object,
                        'type': 'perception'
                    },
                    {
                        'action': 'grasp_object',
                        'target': target_object,
                        'type': 'manipulation'
                    }
                ]
            }
        elif task == 'place':
            target_object = parsed_command['target_object']
            target_location = parsed_command['target_location']
            plan = {
                'task_type': 'manipulation',
                'steps': [
                    {
                        'action': 'move_to_location',
                        'target': target_location,
                        'type': 'navigation'
                    },
                    {
                        'action': 'place_object',
                        'target': target_object,
                        'location': target_location,
                        'type': 'manipulation'
                    }
                ]
            }
        elif task == 'move_arm':
            target_pose = parsed_command['target_pose']
            plan = {
                'task_type': 'manipulation',
                'steps': [
                    {
                        'action': 'move_arm_to_pose',
                        'target_pose': target_pose,
                        'type': 'manipulation'
                    }
                ]
            }
        else:
            return None
            
        return plan

    def plan_high_level_task(self, parsed_command):
        """
        Plan high-level, unstructured tasks
        """
        # For unstructured commands, use LLM to break down into subtasks
        # In this simplified version, we'll just return a placeholder
        return {
            'task_type': 'high_level',
            'steps': [
                {
                    'action': 'interpret_and_execute',
                    'command': parsed_command['command'],
                    'type': 'high_level'
                }
            ]
        }

    def execute_action_plan(self, plan):
        """
        Execute the planned sequence of actions
        """
        steps = plan['steps']
        task_type = plan['task_type']
        
        self.get_logger().info(f'Executing {task_type} task with {len(steps)} steps')
        
        for step in steps:
            success = self.execute_single_action(step)
            if not success:
                self.get_logger().error(f'Action failed: {step}')
                return False  # Stop execution if any action fails
        
        return True

    def execute_single_action(self, action):
        """
        Execute a single action based on its type
        """
        action_type = action['type']
        
        if action_type == 'navigation':
            return self.execute_navigation_action(action)
        elif action_type == 'manipulation':
            return self.execute_manipulation_action(action)
        elif action_type == 'perception':
            return self.execute_perception_action(action)
        elif action_type == 'high_level':
            return self.execute_high_level_action(action)
        else:
            self.get_logger().error(f'Unknown action type: {action_type}')
            return False

    def execute_navigation_action(self, action):
        """
        Execute navigation action
        """
        # For this example, we'll just publish a simple movement command
        # In practice, use navigation stack (Nav2) or similar
        target = action['target']
        
        # Calculate movement command based on target
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward at 0.5 m/s
        cmd.angular.z = 0.0  # No rotation for now
        
        # Publish for 2 seconds (simplified approach)
        for _ in range(20):  # 20 iterations * 0.1s = 2 seconds
            self.nav_cmd_pub.publish(cmd)
            time.sleep(0.1)
        
        # Stop robot
        stop_cmd = Twist()
        self.nav_cmd_pub.publish(stop_cmd)
        
        return True

    def execute_manipulation_action(self, action):
        """
        Execute manipulation action
        """
        action_name = action['action']
        
        if action_name == 'grasp_object':
            # Publish joint trajectory to grasp the object
            trajectory = self.create_grasp_trajectory()
            self.manip_cmd_pub.publish(trajectory)
        elif action_name == 'place_object':
            # Publish joint trajectory to release the object
            trajectory = self.create_place_trajectory()
            self.manip_cmd_pub.publish(trajectory)
        elif action_name == 'move_arm_to_pose':
            # Move arm to specific pose
            target_pose = action['target_pose']
            trajectory = self.create_pose_trajectory(target_pose)
            self.manip_cmd_pub.publish(trajectory)
        else:
            self.get_logger().warn(f'Unknown manipulation action: {action_name}')
            return False
        
        return True

    def create_grasp_trajectory(self):
        """
        Create joint trajectory for grasping
        """
        # Simplified trajectory for demonstration
        trajectory = JointTrajectory()
        trajectory.joint_names = ['left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint']
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [1.0, 0.5, 0.2]  # Example joint angles
        point.velocities = [0.0, 0.0, 0.0]
        point.effort = []
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000  # 200ms
        
        trajectory.points = [point]
        return trajectory

    def create_place_trajectory(self):
        """
        Create joint trajectory for placing
        """
        # Simplified trajectory for demonstration
        trajectory = JointTrajectory()
        trajectory.joint_names = ['left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint']
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0]  # Return to default position
        point.velocities = [0.0, 0.0, 0.0]
        point.effort = []
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000  # 200ms
        
        trajectory.points = [point]
        return trajectory

    def create_pose_trajectory(self, target_pose):
        """
        Create joint trajectory for moving to a specific pose
        """
        # Simplified trajectory for demonstration
        trajectory = JointTrajectory()
        trajectory.joint_names = ['left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint']
        
        # Based on target pose, determine joint angles (simplified)
        positions = [0.0, 0.0, 0.0]  # Default
        if target_pose.get('position') == 'up':
            positions = [0.5, 0.5, 0.5]  # Lift arm up
        elif target_pose.get('position') == 'down':
            positions = [-0.5, -0.5, -0.5]  # Lower arm down
        elif target_pose.get('position') == 'forward':
            positions = [0.0, 1.0, 0.0]  # Extend arm forward
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0, 0.0, 0.0]
        point.effort = []
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500000000  # 500ms
        
        trajectory.points = [point]
        return trajectory

    def execute_perception_action(self, action):
        """
        Execute perception action (e.g., locate object)
        """
        # In practice, this would involve complex perception pipelines
        # For this example, we'll just log the action
        self.get_logger().info(f'Performing perception action: {action}')
        return True

    def execute_high_level_action(self, action):
        """
        Execute high-level, interpreted action
        """
        # For high-level actions, we might need to call additional services
        # or break them down further using LLM assistance
        self.get_logger().info(f'Executing high-level action: {action["command"]}')
        return True


def main(args=None):
    rclpy.init(args=args)
    action_system = ActionGenerationSystem()
    
    try:
        rclpy.spin(action_system)
    except KeyboardInterrupt:
        pass
    finally:
        action_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Implementing Uncertainty Handling

Here's an example of how to handle uncertainty during action execution:

```python
# uncertainty_handling.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from action_generation_system import ActionGenerationSystem
import numpy as np
import random


class UncertaintyHandler(Node):
    def __init__(self):
        super().__init__('uncertainty_handler')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.vision_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.vision_callback,
            10)
        
        # Integration with action generation system
        self.action_system = ActionGenerationSystem()
        
        # Uncertainty tracking
        self.uncertainty_buffer = {}
        self.obstacle_distances = []
        self.vision_quality = 1.0  # 1.0 = perfect, 0.0 = no vision
        
        # Create timer for monitoring and adaptation
        self.monitor_timer = self.create_timer(0.1, self.monitor_environment)
        
        self.get_logger().info('Uncertainty Handler initialized')

    def scan_callback(self, msg):
        """
        Process laser scan data for obstacle detection
        """
        # Filter out invalid ranges
        valid_ranges = [r for r in msg.ranges if 0.0 < r < 10.0]
        if valid_ranges:
            self.obstacle_distances = valid_ranges

    def vision_callback(self, msg):
        """
        Process vision data to assess quality and detect objects
        """
        # For this example, we'll just assess image quality
        # In practice, this would involve object detection and tracking
        
        # Simulate vision quality assessment
        # Lower quality if image is too dark or too bright
        vision_quality = self.assess_vision_quality(msg)
        self.vision_quality = vision_quality

    def assess_vision_quality(self, image_msg):
        """
        Assess the quality of the captured image
        """
        # This is a simplified assessment
        # In practice, use more sophisticated image quality metrics
        
        # For simulation purposes, return a random quality value
        # In real implementation, analyze image brightness, contrast, etc.
        return random.uniform(0.7, 1.0)

    def monitor_environment(self):
        """
        Monitor the environment for changes and uncertainties
        """
        # Assess different types of uncertainties
        perception_uncertainty = self.assess_perception_uncertainty()
        action_uncertainty = self.assess_action_uncertainty()
        model_uncertainty = self.assess_model_uncertainty()
        
        # Update uncertainty buffer
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.uncertainty_buffer[current_time] = {
            'perception': perception_uncertainty,
            'action': action_uncertainty,
            'model': model_uncertainty
        }
        
        # Log uncertainty levels if they exceed thresholds
        if perception_uncertainty > 0.5:
            self.get_logger().warn(f'High perception uncertainty: {perception_uncertainty:.3f}')
        if action_uncertainty > 0.5:
            self.get_logger().warn(f'High action uncertainty: {action_uncertainty:.3f}')

    def assess_perception_uncertainty(self):
        """
        Assess uncertainty in perception
        """
        # Combine different factors
        vision_uncertainty = 1.0 - self.vision_quality
        
        # Calculate obstacle detection uncertainty based on distance
        if self.obstacle_distances:
            min_distance = min(self.obstacle_distances) if self.obstacle_distances else float('inf')
            # Higher uncertainty when obstacles are close
            obstacle_uncertainty = max(0, 1 - min_distance / 2.0)  # 0 at 2m, 1 at 0m
        else:
            obstacle_uncertainty = 0.5  # Default uncertainty if no data
        
        # Combine uncertainties (weighted average)
        combined_uncertainty = 0.6 * vision_uncertainty + 0.4 * obstacle_uncertainty
        return min(1.0, combined_uncertainty)  # Clamp to [0, 1]

    def assess_action_uncertainty(self):
        """
        Assess uncertainty in action execution
        """
        # For this example, return a simulated value
        # In practice, this would be based on robot kinematics, dynamics, etc.
        return random.uniform(0.1, 0.3)

    def assess_model_uncertainty(self):
        """
        Assess uncertainty in environment and robot models
        """
        # For this example, return a simulated value
        # In practice, this would be based on model accuracy, environment changes, etc.
        return random.uniform(0.05, 0.15)

    def adapt_to_uncertainty(self, base_action):
        """
        Adapt the base action based on current uncertainty levels
        """
        # Get current uncertainty levels
        perception_uncertainty = self.assess_perception_uncertainty()
        action_uncertainty = self.assess_action_uncertainty()
        
        # Modify the action based on uncertainty
        adapted_action = base_action.copy()
        
        # If perception uncertainty is high, move more cautiously
        if perception_uncertainty > 0.5:
            adapted_action['speed_multiplier'] = 0.5  # Move at half speed
            adapted_action['check_frequency'] = 2  # Check environment more frequently
        else:
            adapted_action['speed_multiplier'] = 1.0
        
        # If action uncertainty is high, add safety margins
        if action_uncertainty > 0.3:
            if 'target_distance' in adapted_action:
                # Add safety margin to target distance
                adapted_action['target_distance'] += 0.2
        
        # For navigation actions, consider adding more careful planning
        if adapted_action.get('action_type') == 'navigation':
            if perception_uncertainty > 0.6:
                adapted_action['planning_mode'] = 'cautious'  # Use safer, longer paths
            else:
                adapted_action['planning_mode'] = 'efficient'
        
        return adapted_action

    def execute_with_uncertainty_handling(self, action_plan):
        """
        Execute an action plan while handling uncertainty
        """
        for step in action_plan['steps']:
            # Adapt the action based on current uncertainty
            adapted_action = self.adapt_to_uncertainty(step)
            
            # Execute the adapted action
            success = self.execute_single_adapted_action(adapted_action)
            
            if not success:
                self.get_logger().error(f'Action failed: {adapted_action}')
                
                # Try alternative action or recovery behavior
                recovery_success = self.execute_recovery_action(adapted_action)
                
                if not recovery_success:
                    self.get_logger().error('Recovery failed, aborting task')
                    return False
            
            # Monitor environment after each action
            # In practice, this would be continuous monitoring
            self.monitor_environment()
        
        return True

    def execute_single_adapted_action(self, adapted_action):
        """
        Execute a single action with uncertainty adaptations
        """
        # Apply speed multiplier if present
        speed_mult = adapted_action.get('speed_multiplier', 1.0)
        
        if adapted_action['type'] == 'navigation':
            # Execute navigation with adapted parameters
            cmd = Twist()
            cmd.linear.x = 0.5 * speed_mult  # Apply speed multiplier
            
            # Execute for appropriate duration
            duration = adapted_action.get('duration', 1.0) / speed_mult
            self.execute_timed_command(cmd, duration)
            
        elif adapted_action['type'] == 'manipulation':
            # Execute manipulation with added safety checks
            # For this example, just log the action
            self.get_logger().info(f'Executing manipulation: {adapted_action}')
        
        return True

    def execute_timed_command(self, cmd, duration):
        """
        Execute a command for a specific duration
        """
        # Publish command multiple times over duration
        steps = int(duration / 0.1)  # At 10Hz
        for _ in range(steps):
            self.cmd_vel_pub.publish(cmd)
            # In practice, monitor sensors during execution
            self.monitor_environment()
            time.sleep(0.1)
        
        # Stop robot at the end
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def execute_recovery_action(self, failed_action):
        """
        Execute a recovery action when the original action fails
        """
        self.get_logger().info(f'Attempting recovery from failed action: {failed_action}')
        
        # Recovery strategies could include:
        # - Retrying the action
        # - Using alternative approach
        # - Asking for human assistance
        # - Aborting the task
        
        # For this example, we'll just log and return success
        self.get_logger().info('Recovery action executed successfully')
        return True


def main(args=None):
    rclpy.init(args=args)
    uncertainty_handler = UncertaintyHandler()
    
    try:
        rclpy.spin(uncertainty_handler)
    except KeyboardInterrupt:
        pass
    finally:
        uncertainty_handler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Implementing a Complete Vision-Language-Action System

Here's an example that integrates all components into a complete VLA system:

```python
# complete_vla_system.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState, LaserScan
from geometry_msgs.msg import Twist, Pose, PointStamped
from action_generation_system import ActionGenerationSystem
from uncertainty_handling import UncertaintyHandler
from cv_bridge import CvBridge
import numpy as np
import json
from typing import Dict, Any


class CompleteVLASystem(Node):
    def __init__(self):
        super().__init__('complete_vla_system')
        
        # Initialize components
        self.action_system = ActionGenerationSystem()
        self.uncertainty_handler = UncertaintyHandler()
        self.bridge = CvBridge()
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/vla_system_status', 10)
        self.debug_pub = self.create_publisher(String, '/vla_system_debug', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            '/natural_language_commands',
            self.command_callback,
            10)
        
        self.vision_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.vision_callback,
            10)
        
        self.permanent_subscriptions()
        
        # Internal state
        self.conversation_history = []
        self.current_task = None
        self.system_status = 'idle'
        
        # Create main processing timer
        self.main_timer = self.create_timer(0.05, self.main_processing_loop)
        
        self.get_logger().info('Complete VLA System initialized')

    def permanent_subscriptions(self):
        """
        Initialize all permanent subscriptions
        """
        # These are the same subscriptions as in the component nodes
        self.create_subscription(
            String,
            '/natural_language_commands',
            self.command_callback,
            10)
        
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.vision_callback,
            10)
        
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

    def command_callback(self, msg):
        """
        Receive and process natural language commands
        """
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        # Update system status
        self.system_status = 'processing_command'
        
        # Process the command using the action system
        self.action_system.current_language_command = command
        
        # Add to conversation history
        self.conversation_history.append({
            'timestamp': self.get_clock().now().to_msg(),
            'type': 'command',
            'content': command
        })

    def vision_callback(self, msg):
        """
        Process vision data
        """
        # Pass vision data to the action system
        self.action_system.current_vision_data = msg

    def joint_state_callback(self, msg):
        """
        Process joint state data
        """
        # Pass joint state to the action system
        self.action_system.current_joint_states = msg

    def scan_callback(self, msg):
        """
        Process laser scan data
        """
        # Pass scan data to the uncertainty handler
        self.uncertainty_handler.scan_callback(msg)

    def main_processing_loop(self):
        """
        Main processing loop that coordinates all components
        """
        # Process any pending commands
        if self.action_system.current_language_command:
            self.system_status = 'planning'
            
            # Parse the command
            parsed_command = self.action_system.parse_language_command(
                self.action_system.current_language_command)
            
            if parsed_command:
                # Plan the task
                action_plan = self.action_system.plan_task(parsed_command)
                
                if action_plan:
                    self.system_status = 'executing'
                    
                    # Execute the plan with uncertainty handling
                    success = self.uncertainty_handler.execute_with_uncertainty_handling(action_plan)
                    
                    if success:
                        self.system_status = 'completed'
                        self.get_logger().info('Task completed successfully')
                        
                        # Clear the command after successful execution
                        self.action_system.current_language_command = None
                        
                        # Add to conversation history
                        self.conversation_history.append({
                            'timestamp': self.get_clock().now().to_msg(),
                            'type': 'action_result',
                            'content': 'success',
                            'command': self.action_system.current_language_command
                        })
                    else:
                        self.system_status = 'failed'
                        self.get_logger().error('Task execution failed')
                        
                        # Add to conversation history
                        self.conversation_history.append({
                            'timestamp': self.get_clock().now().to_msg(),
                            'type': 'action_result',
                            'content': 'failed',
                            'command': self.action_system.current_language_command
                        })
                else:
                    self.system_status = 'planning_failed'
            else:
                self.system_status = 'parsing_failed'
        else:
            self.system_status = 'idle'
        
        # Publish system status
        status_msg = String()
        status_msg.data = json.dumps({
            'status': self.system_status,
            'timestamp': self.get_clock().now().to_msg(),
            'command_queue_size': len(self.action_system.command_history)
        })
        self.status_pub.publish(status_msg)

        # Periodically log system information
        if self.get_clock().now().nanoseconds % 10000000000 < 50000000:  # Every 10 seconds
            self.get_logger().info(f'VLA System Status: {self.system_status}')
    
    def get_system_state(self) -> Dict[str, Any]:
        """
        Get the current state of the VLA system
        """
        return {
            'status': self.system_status,
            'conversation_history': self.conversation_history[-5:],  # Last 5 interactions
            'active_task': self.current_task,
            'command_queue_size': len(self.action_system.command_history),
            'vision_quality': getattr(self.uncertainty_handler, 'vision_quality', 1.0)
        }


def main(args=None):
    rclpy.init(args=args)
    vla_system = CompleteVLASystem()
    
    try:
        rclpy.spin(vla_system)
    except KeyboardInterrupt:
        pass
    finally:
        vla_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercises

1. **Logical Exercise**: Analyze the challenges of action grounding in robotics. How does the process of converting high-level language commands into specific robot actions differ from traditional programming approaches?

2. **Conceptual Exercise**: Explain the concept of affordances in robotics and how they relate to action generation. How do robots determine what actions are possible in a given environment?

3. **Implementation Exercise**: Create a complete vision-language-action system that can process a command like "Go to the kitchen and bring me the red cup from the table." Implement the full pipeline: language understanding, visual grounding, task planning, and action execution with uncertainty handling.

## Summary

This chapter covered action generation and execution in Vision-Language-Action systems. We explored how high-level language commands are translated into executable robot behaviors, the importance of handling uncertainty during execution, and how to integrate perception, language, and action systems into a cohesive framework.

The integration of language understanding with action execution enables robots to respond flexibly to human commands in dynamic environments. As these systems continue to improve, they will enable more natural and intuitive human-robot interaction, making robots more useful and accessible.

## References
- [RT-2: Vision-Language-Action Models for Robot Manipulation](https://arxiv.org/abs/2307.15818)
- [Language Models as Zero-Shot Planners](https://arxiv.org/abs/2201.07207)
- [Vision-Language Models for Grounded Robot Navigation](https://arxiv.org/abs/2109.01666)