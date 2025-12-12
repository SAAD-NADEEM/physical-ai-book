---
title: Chapter 3 - Robot Manipulation and Control
description: Understanding robot kinematics, dynamics, and control for manipulation tasks
keywords: [manipulation, kinematics, dynamics, control, robotics]
sidebar_position: 4
module_ref: module-1-ros2
prerequisites: ["Module 1, Chapter 2"]
learning_objectives: ["Implement forward and inverse kinematics", "Control robotic manipulators", "Plan collision-free paths", "Integrate kinematic models with ROS 2"]
estimated_reading_time: 60
exercises_count: 3
---

# Chapter 3: Robot Manipulation and Control

## Learning Objectives
- Implement forward and inverse kinematics for robotic manipulators
- Control robotic manipulators using ROS 2
- Plan collision-free paths for manipulation tasks
- Integrate kinematic models with ROS 2 using MoveIt!

## Prerequisites
- Understanding of ROS 2 communication patterns (topics, services, actions)
- Basic knowledge of linear algebra and coordinate transformations
- Experience with URDF for robot modeling
- Familiarity with robot control concepts

## Core Concepts

Robot manipulation involves controlling robotic arms and end-effectors to perform tasks like grasping, moving, and placing objects. This requires understanding of kinematics, dynamics, and control methods.

### Forward Kinematics

Forward kinematics computes the position and orientation of the end-effector given joint angles. For a robotic arm with n joints, we calculate the transformation matrix from the base frame to the end-effector frame:

T_0^n = T_0^1(θ_1) * T_1^2(θ_2) * ... * T_(n-1)^n(θ_n)

Where T_i^(i+1) represents the transformation from frame i to frame i+1.

### Inverse Kinematics

Inverse kinematics solves for joint angles that achieve a desired end-effector position and orientation. This is typically more complex than forward kinematics and may have multiple solutions or no solution.

### Robot Control Approaches

1. **Position Control**: Directly command joint positions
2. **Velocity Control**: Command joint velocities
3. **Effort/Torque Control**: Command joint efforts/torques
4. **Cartesian Control**: Control end-effector position in Cartesian space

## Implementation

### Implementing Forward Kinematics

Here's an implementation of forward kinematics calculation for a simple robotic arm:

```python
import numpy as np
import math
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import rclpy
from rclpy.node import Node


class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')
        
        # Publisher for visualizing kinematic chain
        self.link_viz_pub = self.create_publisher(Marker, '/robot_links_viz', 10)
        
        # Timer for visualization
        self.viz_timer = self.create_timer(0.1, self.visualize_kinematics)
        
        # Define DH parameters for a simple 3-DOF arm (for demonstration)
        self.dh_params = [
            {'a': 0.1, 'alpha': math.pi/2, 'd': 0.1, 'theta_offset': 0},
            {'a': 0.5, 'alpha': 0, 'd': 0, 'theta_offset': 0},
            {'a': 0.5, 'alpha': 0, 'd': 0, 'theta_offset': 0}
        ]
        
        # Current joint angles
        self.joint_angles = [0.0, 0.0, 0.0]  # Initially at home position
        
        self.get_logger().info('Forward Kinematics Node initialized')

    def dh_transform(self, a, alpha, d, theta):
        """
        Calculate Denavit-Hartenberg transformation matrix
        """
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        st = math.sin(theta)
        ct = math.cos(theta)
        
        transform = np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        
        return transform

    def calculate_forward_kinematics(self, joint_angles):
        """
        Calculate forward kinematics for the robot arm
        Returns transformation matrix from base to end-effector
        """
        # Validate number of joints
        if len(joint_angles) != len(self.dh_params):
            raise ValueError(f"Expected {len(self.dh_params)} joint angles, got {len(joint_angles)}")
        
        # Initialize transformation matrix as identity
        T_total = np.eye(4)
        
        # Calculate transformation for each joint
        for i, (theta, dh) in enumerate(zip(joint_angles, self.dh_params)):
            # Add joint angle to theta offset
            current_theta = theta + dh['theta_offset']
            
            # Calculate DH transformation for this joint
            T_link = self.dh_transform(dh['a'], dh['alpha'], dh['d'], current_theta)
            
            # Add to total transformation
            T_total = T_total @ T_link
        
        return T_total

    def extract_position_orientation(self, transform_matrix):
        """
        Extract position and orientation from transformation matrix
        """
        # Position is the translation part of the matrix
        position = transform_matrix[:3, 3]
        
        # Orientation is the rotation part of the matrix
        rotation_matrix = transform_matrix[:3, :3]
        
        # Convert rotation matrix to quaternion (simplified)
        # In practice, use transformations library
        trace = np.trace(rotation_matrix)
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2  # S=4*qw
            qw = 0.25 * s
            qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        else:
            # Handle other cases for quaternion calculation
            if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                s = math.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2
                qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                qx = 0.25 * s
                qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                s = math.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2
                qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                qy = 0.25 * s
                qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            else:
                s = math.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2
                qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
                qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                qz = 0.25 * s
        
        return position, [qx, qy, qz, qw]

    def visualize_kinematics(self):
        """
        Visualize the robot kinematic chain
        """
        # Calculate current end-effector position
        transform_matrix = self.calculate_forward_kinematics(self.joint_angles)
        position, orientation = self.extract_position_orientation(transform_matrix)
        
        # Create marker for end-effector
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.ns = "robot_kinematics"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        
        # Set orientation
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        
        # Set scale (0.1m radius sphere)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Set color (blue)
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        
        self.link_viz_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    fk_node = ForwardKinematicsNode()
    
    try:
        rclpy.spin(fk_node)
    except KeyboardInterrupt:
        pass
    finally:
        fk_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Implementing Inverse Kinematics with MoveIt!

Here's an example of using MoveIt! for inverse kinematics and manipulation:

```python
import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from rclpy.action import ActionClient
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetPositionFK, GetPositionIK
import tf_transformations


class MoveItController(Node):
    def __init__(self):
        super().__init__('moveit_controller')
        
        # Action client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, 'move_group')
        
        # Service clients for kinematics
        self.fk_client = self.create_client(GetPositionFK, 'compute_fk')
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        
        # Publisher for commands
        self.command_pub = self.create_publisher(String, '/manipulation_commands', 10)
        
        # Timer for periodic tasks
        self.timer = self.create_timer(0.5, self.periodic_tasks)
        
        # Wait for services to be available
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('FK service not available, waiting again...')
            
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting again...')
        
        self.get_logger().info('MoveIt Controller initialized')

    def move_to_pose(self, target_pose):
        """
        Move the robot to a target pose using MoveIt
        """
        goal_msg = MoveGroup.Goal()
        
        # Set up the goal for the manipulator
        goal_msg.request.group_name = 'manipulator'  # Name of the planning group
        
        # Define the target pose constraint
        pose_constraint = self.create_pose_constraint(target_pose)
        goal_msg.request.goal_constraints.append(pose_constraint)
        
        # Set planning parameters
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 10.0  # seconds
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # Send the goal
        self.move_group_client.wait_for_server()
        future = self.move_group_client.send_goal_async(goal_msg)
        
        # Add a callback for when the goal is accepted/rejected
        future.add_done_callback(self.goal_response_callback)

    def create_pose_constraint(self, target_pose):
        """
        Create a pose constraint for MoveIt
        """
        from moveit_msgs.msg import Constraints
        from geometry_msgs.msg import PoseStamped
        import moveit_msgs.msg as moveit_msgs
        
        # Create constraints object
        constraints = Constraints()
        
        # Add position constraint
        position_constraint = moveit_msgs.PositionConstraint()
        position_constraint.link_name = 'end_effector_link'  # The link to constrain
        
        # Create target pose as PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.pose = target_pose
        
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        
        # Define the workspace region (bounding box)
        position_constraint.constraint_region.primitives.append(SolidPrimitive())
        position_constraint.constraint_region.primitives[0].type = SolidPrimitive.BOX
        position_constraint.constraint_region.primitives[0].dimensions = [0.01, 0.01, 0.01]  # Tight constraint
        
        # Set the pose of the constraint region
        position_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
        
        position_constraint.weight = 1.0
        
        constraints.position_constraints.append(position_constraint)
        
        # Add orientation constraint
        orientation_constraint = moveit_msgs.OrientationConstraint()
        orientation_constraint.link_name = 'end_effector_link'
        orientation_constraint.header.frame_id = 'base_link'
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        
        constraints.orientation_constraints.append(orientation_constraint)
        
        return constraints

    def goal_response_callback(self, future):
        """
        Callback for when the MoveGroup goal is accepted
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        
        # Get the result of the action
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Callback for when the MoveGroup action is completed
        """
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        
        # Check if the motion was successful
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Motion successful!')
        else:
            self.get_logger().error(f'Motion failed with error code: {result.error_code.val}')

    def compute_ik(self, target_pose, group_name='manipulator'):
        """
        Compute inverse kinematics for a target pose
        """
        from moveit_msgs.srv import GetPositionIK
        from moveit_msgs.msg import PositionIKRequest
        
        # Create IK request
        ik_request = PositionIKRequest()
        ik_request.group_name = group_name
        ik_request.pose_stamped.header.frame_id = 'base_link'
        ik_request.pose_stamped.pose = target_pose
        ik_request.timeout.sec = 5  # 5-second timeout
        
        # Create service request
        request = GetPositionIK.Request()
        request.ik_request = ik_request
        
        # Call the service
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_result_callback)

    def ik_result_callback(self, future):
        """
        Callback for when the IK service returns a result
        """
        try:
            response = future.result()
            if response.error_code.val == 1:  # SUCCESS
                self.get_logger().info('IK solution found')
                joint_angles = response.solution.joint_state
                self.get_logger().info(f'Joint angles: {joint_angles.position}')
            else:
                self.get_logger().error(f'IK failed with error code: {response.error_code.val}')
        except Exception as e:
            self.get_logger().error(f'Exception in IK callback: {e}')

    def periodic_tasks(self):
        """
        Perform periodic tasks
        """
        # For demonstration, we'll just call IK for a predefined pose periodically
        pass


def main(args=None):
    rclpy.init(args=args)
    moveit_controller = MoveItController()
    
    # Example: Set a target pose and move to it
    target_pose = Pose()
    target_pose.position.x = 0.5
    target_pose.position.y = 0.2
    target_pose.position.z = 0.6
    
    # Set orientation (identity quaternion for simplicity)
    target_pose.orientation.w = 1.0
    
    # For testing, just call IK for this pose
    moveit_controller.compute_ik(target_pose)
    
    try:
        rclpy.spin(moveit_controller)
    except KeyboardInterrupt:
        pass
    finally:
        moveit_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Implementing Robot Control with Joint Trajectory Controller

Here's an example of controlling a robotic manipulator using joint trajectories:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
import numpy as np
import time


class ManipulatorController(Node):
    def __init__(self):
        super().__init__('manipulator_controller')
        
        # Publisher for joint trajectory commands
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Store current joint states
        self.current_joint_positions = {}
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 
                           'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Timer for sending control commands
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Manipulator Controller initialized')

    def joint_state_callback(self, msg):
        """
        Callback for joint state updates
        """
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joint_positions[name] = msg.position[i]

    def control_loop(self):
        """
        Main control loop for the manipulator
        """
        # For this example, we'll create a simple repetitive motion
        # In a real implementation, this would follow a planned trajectory
        
        current_time = time.time()
        
        # Create oscillating joint positions for demonstration
        joint_positions = {}
        for i, joint_name in enumerate(self.joint_names):
            # Create different oscillation patterns for different joints
            amplitude = 0.2 if i < 3 else 0.1  # Larger amplitude for first 3 joints
            frequency = 0.5 + i * 0.2  # Different frequencies
            offset = i * np.pi / 4  # Phase offset
            
            position = amplitude * np.sin(frequency * current_time + offset)
            joint_positions[joint_name] = position
        
        # Send the trajectory
        self.send_joint_trajectory(joint_positions)

    def send_joint_trajectory(self, joint_positions_dict, duration=2.0):
        """
        Send a joint trajectory command to move to specified joint positions
        """
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = list(joint_positions_dict.keys())
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions_dict.values())
        
        # Initialize velocities and efforts to zero
        point.velocities = [0.0] * len(joint_positions_dict)
        point.accelerations = [0.0] * len(joint_positions_dict)
        point.effort = []
        
        # Set the time for the trajectory point
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration - int(duration)) * 1e9)
        )
        
        trajectory_msg.points = [point]
        trajectory_msg.header = Header()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the trajectory
        self.joint_trajectory_pub.publish(trajectory_msg)

    def move_to_home_position(self):
        """
        Move the manipulator to its home position
        """
        home_positions = {
            'shoulder_pan_joint': 0.0,
            'shoulder_lift_joint': -1.57,
            'elbow_joint': 1.57,
            'wrist_1_joint': -1.57,
            'wrist_2_joint': -1.57,
            'wrist_3_joint': 0.0
        }
        
        self.send_joint_trajectory(home_positions)

    def execute_grasp_motion(self):
        """
        Execute a simplified grasp motion
        """
        # Go to a position above the object
        approach_positions = {
            'shoulder_pan_joint': 0.5,
            'shoulder_lift_joint': -1.0,
            'elbow_joint': 1.2,
            'wrist_1_joint': -1.8,
            'wrist_2_joint': -1.57,
            'wrist_3_joint': 0.0
        }
        
        self.send_joint_trajectory(approach_positions, duration=3.0)
        
        # Move down to grasp position (simplified)
        grasp_positions = approach_positions.copy()
        grasp_positions['shoulder_lift_joint'] = -0.5  # Move down
        
        # Schedule the grasp after the approach
        def send_grasp():
            self.send_joint_trajectory(grasp_positions, duration=2.0)
        
        # In a real implementation, we'd use a timer or action client
        # For this example, just log what would happen
        self.get_logger().info('Grasp motion would be executed')


def main(args=None):
    rclpy.init(args=args)
    controller = ManipulatorController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercises

1. **Logical Exercise**: Compare different approaches to inverse kinematics (analytical vs. numerical methods). What are the advantages and disadvantages of each approach for humanoid robotics applications?

2. **Conceptual Exercise**: Explain the concept of manipulability in robotics and its importance for robot control. How does the manipulability of a robot arm change throughout its workspace?

3. **Implementation Exercise**: Create a complete robotic manipulation system using MoveIt! that can pick up an object from a table and place it at a target location. Implement both path planning and collision avoidance.

## Summary

This chapter covered robot manipulation and control, including forward and inverse kinematics, controlling robotic manipulators, and integrating kinematic models with ROS 2 using MoveIt!. We explored different control approaches and implemented practical examples of kinematic calculations and manipulator control.

Robot manipulation is a fundamental capability for humanoid robots, enabling them to interact with objects in their environment. Understanding kinematics and control methods is essential for developing effective manipulation systems.

## References
- [Robotics, Vision and Control by Peter Corke](https://link.springer.com/book/10.1007/978-3-642-20144-8)
- [MoveIt! Documentation](https://moveit.ros.org/)
- [ROS Control Documentation](https://ros-controls.github.io/control.ros.org/)