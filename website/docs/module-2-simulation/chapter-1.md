---
title: Chapter 1 - Physics Simulation with Gazebo
description: Creating realistic physics simulation environments with Gazebo for robotics applications
keywords: [gazebo, physics, simulation, urdf, robot modeling]
sidebar_position: 2
module_ref: module-2-simulation
prerequisites: ["Module 2, Intro"]
learning_objectives: ["Create realistic physics simulation environments", "Model robots using URDF", "Implement sensor simulation", "Integrate Gazebo with ROS 2"]
estimated_reading_time: 60
exercises_count: 3
---

# Chapter 1: Physics Simulation with Gazebo

## Learning Objectives
- Understand the Gazebo physics engine and its architecture
- Create robot models using URDF (Unified Robot Description Format)
- Implement physics properties, joints, and sensors in Gazebo
- Integrate Gazebo simulation with ROS 2

## Prerequisites
- Basic understanding of robot kinematics and dynamics
- ROS 2 fundamentals (Nodes, Topics, Services)
- Familiarity with 3D modeling concepts
- Understanding of coordinate systems and transformations

## Core Concepts

Gazebo is a powerful physics-based simulation tool that is widely used in the robotics community. It provides realistic simulation of robots in complex environments, incorporating accurate physics, sensor simulation, and rendering capabilities.

### Gazebo Architecture

Gazebo operates on a client-server model with multiple components:

1. **Gazebo Server (gzserver)**: Handles the physics simulation, sensor updates, and model management
2. **Gazebo Client (gzclient)**: Provides the graphical user interface for visualization
3. **Transport Layer**: Manages communication between server and client components
4. **Plugins System**: Extends functionality through dynamic loading of plugins
5. **Physics Engine**: Currently uses Ignition Physics which supports multiple backends (DART, Bullet, ODE)

### Robot Modeling with URDF

URDF (Unified Robot Description Format) is the standard for describing robots in ROS. It defines the robot's structure, kinematics, and dynamics. For Gazebo simulation, URDF files are extended with Gazebo-specific tags.

Here's an example of a humanoid robot URDF description with Gazebo-specific elements:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Base Link (trunk) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Joint connecting head to base -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.0075" ixy="0.0" ixz="0.0" iyy="0.0075" iyz="0.0" izz="0.00025"/>
    </inertial>
  </link>

  <!-- Joint connecting left arm to base -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <!-- Gazebo-specific tags -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/White</material>
  </gazebo>

  <gazebo reference="left_upper_arm">
    <material>Gazebo/Red</material>
  </gazebo>

  <!-- ROS Control plugin -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
    </plugin>
  </gazebo>

</robot>
```

### Physics Properties in Gazebo

Gazebo allows you to define detailed physics properties for each link in your robot:

1. **Mass and Inertia**: Defines how the object responds to forces and torques
2. **Collision Properties**: Determines how objects interact when they collide
3. **Surface Properties**: Controls friction, bounciness, and other contact properties
4. **Dynamics Properties**: Configures joint limits, effort, and velocity constraints

### Sensor Simulation

Gazebo provides a rich set of sensor simulations that closely match real hardware:

- **Camera Sensors**: Simulates RGB, depth, and stereo cameras
- **Lidar Sensors**: Simulates 2D and 3D LiDAR systems
- **IMU (Inertial Measurement Unit)**: Provides acceleration and angular velocity data
- **Force/Torque Sensors**: Measures forces and torques at joints
- **GPS Sensors**: Provides position and velocity information
- **Contact Sensors**: Detects physical contact with other objects

## Implementation

### Setting up a Gazebo Simulation Environment

Here's how to create a basic Gazebo simulation with a robot model:

1. **Create a launch file** to start Gazebo with your robot model:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_sim = get_package_share_directory('robot_simulation')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Start Gazebo server and client
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')))

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))

    # Spawn the robot in Gazebo
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'humanoid_robot',
                   '-file', os.path.join(pkg_robot_sim, 'models', 'humanoid_robot.urdf'),
                   '-x', '0.0', '-y', '0.0', '-z', '1.0'],
        output='screen')

    # Create the launch description
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_entity_cmd)

    return ld
```

2. **Create a robot controller node** to interact with the simulated robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import math


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Create publisher for joint trajectory commands
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, '/humanoid_robot/joint_trajectory_controller/joint_trajectory', 10)
        
        # Create subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/humanoid_robot/joint_states', self.joint_state_callback, 10)
        
        # Timer for sending commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Store current joint states
        self.current_joint_states = JointState()
        
        # Initialize trajectory message
        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.joint_names = [
            'left_shoulder_joint', 'neck_joint'
        ]
        
        self.get_logger().info('Robot Controller initialized')

    def joint_state_callback(self, msg):
        self.current_joint_states = msg

    def timer_callback(self):
        # Create a trajectory point
        point = JointTrajectoryPoint()
        
        # Generate a simple oscillating motion
        current_time = self.get_clock().now().nanoseconds / 1e9
        left_arm_pos = 0.5 * math.sin(current_time)
        neck_pos = 0.3 * math.sin(current_time * 0.7)
        
        point.positions = [left_arm_pos, neck_pos]
        point.velocities = [0.0, 0.0]  # For simplicity, zero velocity
        point.effort = []  # Empty effort for position control
        
        # Set timing
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50000000  # 50ms
        
        # Set the trajectory
        self.trajectory_msg.points = [point]
        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the trajectory
        self.joint_trajectory_pub.publish(self.trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Custom Gazebo Plugin

For more advanced simulation, you can create custom Gazebo plugins. Here's a simple example of a custom plugin:

```cpp
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math.hh>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

namespace gazebo
{
  class CustomController : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the model pointer for convenience
      this->model = _parent;

      // Get the first joint (you might need to adjust this)
      this->joint = this->model->GetJoint("left_shoulder_joint");

      // Initialize ROS if not already initialized
      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "gazebo_custom_controller",
                  ros::init_options::NoSigintHandler);
      }

      // Create a ROS node
      this->rosNode.reset(new ros::NodeHandle("gazebo_ros"));

      // Create a subscriber on the /cmd topic
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/cmd",
            1,
            boost::bind(&CustomController::SetPosition, this, _1),
            ros::VoidPtr(), &this->queue);

      this->sub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread
      this->callbackQueueThread =
        boost::thread(boost::bind(&CustomController::QueueThread, this));

      // Listen to the update event (every simulation iteration)
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&CustomController::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // Apply a force to the joint if needed
      // This is where you would implement your control logic
    }

    // Set the joint position based on ROS topic
    private: void SetPosition(const std_msgs::Float32::ConstPtr& _msg)
    {
      // Set the joint position to the value from ROS topic
      this->joint->SetPosition(0, _msg->data);
    }

    // ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->queue.callAvailable(ros::WallDuration(timeout));
      }
    }

    private: physics::ModelPtr model;
    private: physics::JointPtr joint;
    private: event::ConnectionPtr updateConnection;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber sub;
    private: ros::CallbackQueue queue;
    private: boost::thread callbackQueueThread;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CustomController)
}
```

## Exercises

1. **Logical Exercise**: Compare and contrast the physics simulation capabilities of Gazebo with alternatives like PyBullet or MuJoCo. Analyze the advantages and disadvantages of each for humanoid robotics applications.

2. **Conceptual Exercise**: Explain the concept of "domain randomization" in simulation and its importance for bridging the reality gap. How can this technique be applied to humanoid robot training?

3. **Implementation Exercise**: Create a Gazebo simulation environment with a simple humanoid robot model. Implement a controller that makes the robot perform a basic movement pattern (e.g., waving its arm). Include at least one sensor (e.g., IMU, camera, or LiDAR) in your simulation and visualize the sensor data.

## Summary

This chapter introduced Gazebo as a powerful physics-based simulation environment for robotics. We covered robot modeling with URDF, physics properties, sensor simulation, and the integration of Gazebo with ROS 2. We also demonstrated how to create launch files for simulation environments and how to implement custom controllers and plugins.

Simulation is a critical tool in robotics development, allowing for safe, cost-effective testing and validation of robotic systems before deployment on physical hardware. Understanding how to create realistic simulation environments is essential for any robotics engineer.

## References
- [Gazebo Documentation](http://gazebosim.org/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [ROS Control Documentation](https://ros-controls.github.io/control.ros.org/)