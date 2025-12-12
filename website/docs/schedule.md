---
title: Course Schedule
description: Weekly schedule for the Physical AI and Humanoid Robotics course
keywords: [schedule, timeline, robotics, course plan]
sidebar_position: 2
---

# Course Schedule: Physical AI and Humanoid Robotics

This page provides the detailed weekly schedule for the 13-week graduate course on Physical AI and Humanoid Robotics.

## Week 1: Introduction to ROS 2 and Robot Architecture

### Learning Objectives
- Understand the architecture of ROS 2
- Identify core components of a ROS 2 system
- Recognize the differences between ROS 1 and ROS 2
- Set up the development environment

### Content
- ROS 2 architecture overview
- DDS (Data Distribution Service) concepts
- Nodes, packages, and workspaces
- Development environment setup

### Exercises
1. **Logical Exercise**: Compare and contrast ROS 1 and ROS 2 architectures
2. **Conceptual Exercise**: Explain the role of DDS in ROS 2 communication
3. **Implementation Exercise**: Create a basic ROS 2 workspace and run sample nodes

### Resources
- [Official ROS 2 Documentation](https://docs.ros.org/)
- [DDS Standard Overview](https://www.omg.org/spec/DDS/)

---

## Week 2: Nodes, Topics, Services, and Actions

### Learning Objectives
- Implement ROS 2 nodes in Python and C++
- Understand the communication patterns in ROS 2
- Create publishers, subscribers, services, and actions
- Debug communication issues

### Content
- Node creation and lifecycle
- Topics and message passing
- Services for request/reply communication
- Actions for long-running tasks
- Quality of Service (QoS) settings

### Exercises
1. **Logical Exercise**: Analyze when to use topics vs services vs actions
2. **Conceptual Exercise**: Explain Quality of Service (QoS) settings
3. **Implementation Exercise**: Create a publisher and subscriber for sensor data

### Resources
- [ROS 2 Tutorials](https://docs.ros.org/en/rolling/Tutorials.html)
- [ROS 2 Communication Patterns](https://docs.ros.org/en/rolling/Concepts/About-Topics-Services-Actions.html)

---

## Week 3: Robot Manipulation and Control

### Learning Objectives
- Understand robot kinematics and dynamics
- Implement forward and inverse kinematics
- Control robotic manipulators
- Plan collision-free paths

### Content
- Forward and inverse kinematics
- Jacobian matrices
- Robot kinematic chains
- MoveIt! for motion planning
- Control interfaces

### Exercises
1. **Logical Exercise**: Analyze the differences between position and velocity control
2. **Conceptual Exercise**: Explain the singularity problem in inverse kinematics
3. **Implementation Exercise**: Control a simulated robotic arm to reach target positions

### Resources
- [MoveIt! Documentation](https://moveit.ros.org/)
- [Robotics Toolbox for Python](https://pypi.org/project/robotics-toolbox-python/)

---

## Week 4: Navigation and Path Planning

### Learning Objectives
- Implement navigation systems for mobile robots
- Plan collision-free paths in known and unknown environments
- Integrate perception with navigation
- Evaluate navigation performance

### Content
- Robot pose estimation and localization
- Map representation and SLAM
- Path planning algorithms (A*, Dijkstra, RRT)
- Navigation stack configuration
- Obstacle avoidance

### Exercises
1. **Logical Exercise**: Compare global vs local path planning approaches
2. **Conceptual Exercise**: Explain the relationship between SLAM and navigation
3. **Implementation Exercise**: Configure and test the ROS 2 navigation stack

### Resources
- [Navigation2 Documentation](https://navigation.ros.org/)
- [SLAM in Robotics](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)

---

## Week 5: Multi-Robot Systems and Coordination

### Learning Objectives
- Design systems for coordinating multiple robots
- Implement communication protocols between robots
- Address challenges in multi-robot coordination
- Evaluate multi-robot system performance

### Content
- Multi-robot communication patterns
- Task allocation and coordination
- Distributed decision making
- Conflict resolution in multi-robot systems

### Exercises
1. **Logical Exercise**: Analyze the challenges of coordination in dynamic environments
2. **Conceptual Exercise**: Compare centralized vs decentralized coordination
3. **Implementation Exercise**: Implement a simple multi-robot coordination scenario

### Resources
- [Multi-robot Systems Literature](https://ieeexplore.ieee.org/document/6343147)
- [Distributed Robotics](https://www.sciencedirect.com/topics/engineering/distributed-robotics)

---

## Week 6: Physics Simulation with Gazebo

### Learning Objectives
- Create realistic simulation environments
- Integrate robots with physics simulation
- Validate robot designs through simulation
- Benchmark robot performance in simulation

### Content
- Gazebo physics engine
- Robot modeling and URDF files
- Simulation plugins and sensors
- Integration with ROS 2

### Exercises
1. **Logical Exercise**: Analyze the differences between simulation and reality
2. **Conceptual Exercise**: Explain the concept of "reality gap" in robotics
3. **Implementation Exercise**: Create a simulation environment for a robot and test navigation

### Resources
- [Gazebo Documentation](http://gazebosim.org/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)

---

## Week 7: Unity Integration and Visual Simulation

### Learning Objectives
- Integrate ROS 2 with Unity for visual simulation
- Create custom visual environments in Unity
- Synchronize Unity visuals with ROS 2 simulation
- Leverage Unity's rendering capabilities for robotics

### Content
- Unity-ROS bridge
- Visual simulation scenarios
- Camera and sensor visualization
- High-quality rendering for perception tasks

### Exercises
1. **Logical Exercise**: Compare Gazebo and Unity for robotics simulation
2. **Conceptual Exercise**: Explain the concept of photo-realistic simulation
3. **Implementation Exercise**: Create a Unity scene and connect it with ROS 2

### Resources
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS# Unity Package](https://github.com/siemens/ros-sharp)

---

## Week 8: Introduction to Isaac Sim and Isaac ROS

### Learning Objectives
- Set up NVIDIA Isaac Sim and Isaac ROS
- Understand the NVIDIA Isaac ecosystem
- Create simulation environments with Isaac Sim
- Leverage NVIDIA hardware for robotics

### Content
- Isaac Sim architecture
- GPU-accelerated simulation
- Isaac ROS components
- Hardware acceleration for robotics

### Exercises
1. **Logical Exercise**: Analyze the advantages of GPU acceleration in robotics
2. **Conceptual Exercise**: Explain the relationship between Isaac Sim and Isaac ROS
3. **Implementation Exercise**: Run a sample simulation in Isaac Sim

### Resources
- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)

---

## Week 9: Perception and Sensing with Isaac

### Learning Objectives
- Implement perception systems using Isaac
- Leverage Isaac's perception capabilities
- Integrate sensors with Isaac Sim
- Process perception data for robot control

### Content
- Isaac perception components
- Sensor simulation in Isaac Sim
- Computer vision processing
- Sensor fusion techniques

### Exercises
1. **Logical Exercise**: Compare perception in simulation vs real environments
2. **Conceptual Exercise**: Explain sensor fusion in robotics
3. **Implementation Exercise**: Implement a perception pipeline in Isaac

### Resources
- [Isaac ROS Perception Packages](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_perceptor/index.html)
- [Computer Vision in Robotics](https://www.cs.cmu.edu/~ILIM/projects/CMUcam/)

---

## Week 10: Navigation and Manipulation Frameworks

### Learning Objectives
- Implement navigation systems using Isaac frameworks
- Control manipulation tasks with Isaac
- Integrate perception with navigation and manipulation
- Evaluate performance of Isaac-based systems

### Content
- Isaac navigation components
- Isaac manipulation frameworks
- Perception-action loops
- Performance evaluation metrics

### Exercises
1. **Logical Exercise**: Analyze the integration challenges in perception-action loops
2. **Conceptual Exercise**: Explain the role of simulation in developing navigation systems
3. **Implementation Exercise**: Create a navigation task using Isaac frameworks

### Resources
- [Isaac ROS Navigation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_navigation/index.html)
- [Manipulation Frameworks](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_manipulation/index.html)

---

## Week 11: Vision-Language Models for Robotics

### Learning Objectives
- Understand vision-language models in robotics context
- Integrate VLMs with robotic systems
- Process natural language commands
- Interpret visual information with language models

### Content
- Vision-language models overview
- Integration with robotic systems
- Natural language processing for robots
- Multimodal perception

### Exercises
1. **Logical Exercise**: Analyze the challenges of grounding language in robotic actions
2. **Conceptual Exercise**: Explain the difference between captioning and instruction following
3. **Implementation Exercise**: Create a system that interprets language commands in visual context

### Resources
- [CLIP Model](https://openai.com/research/clip)
- [Vision-Language Models Survey](https://arxiv.org/abs/2209.03430)

---

## Week 12: Action Generation and Execution

### Learning Objectives
- Generate robot actions based on vision-language inputs
- Plan and execute complex actions
- Integrate perception, language, and action systems
- Handle uncertainty in action execution

### Content
- Action generation models
- Planning in vision-language-action systems
- Uncertainty handling
- Execution monitoring and adaptation

### Exercises
1. **Logical Exercise**: Analyze the challenges of action grounding
2. **Conceptual Exercise**: Explain the concept of affordances in robotics
3. **Implementation Exercise**: Implement an action execution system based on visual-language input

### Resources
- [Language Grounding in Robotics](https://arxiv.org/abs/2107.14246)
- [Action Planning Literature](https://www.cs.ubc.ca/~poole/eaia/Chapters/Chapter9.pdf)

---

## Week 13: Capstone Project Integration

### Learning Objectives
- Integrate all modules into a comprehensive system
- Demonstrate end-to-end functionality
- Evaluate system performance
- Present project outcomes

### Content
- Integration of all learned components
- System architecture and design
- Performance evaluation
- Project presentation and documentation

### Exercises
1. **Logical Exercise**: Analyze the design decisions made during integration
2. **Conceptual Exercise**: Explain the challenges of system integration in robotics
3. **Implementation Exercise**: Complete and demonstrate the capstone project

### Resources
- [Project Presentation Guidelines](https://example.com/presentation-guidelines)
- [System Integration Best Practices](https://example.com/integration-practices)