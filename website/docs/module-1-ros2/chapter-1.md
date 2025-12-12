---
title: Chapter 1 - Introduction to ROS 2 and Robot Architecture
description: Overview of ROS 2 architecture and concepts for humanoid robotics
keywords: [ros2, robotics, middleware, architecture]
sidebar_position: 2
module_ref: module-1-ros2
prerequisites: []
learning_objectives: ["Understand ROS 2 architecture", "Identify core components", "Recognize differences between ROS 1 and ROS 2"]
estimated_reading_time: 30
exercises_count: 3
---

# Chapter 1: Introduction to ROS 2 and Robot Architecture

## Learning Objectives
- Understand the architecture of ROS 2
- Identify the core components of a ROS 2 system
- Recognize the differences between ROS 1 and ROS 2
- Set up the ROS 2 development environment

## Prerequisites
- Basic understanding of robotics concepts
- Programming experience in Python or C++
- Familiarity with command-line interfaces

## Core Concepts

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot applications. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. Unlike the original ROS, ROS 2 is built on DDS (Data Distribution Service), which provides a more robust and scalable architecture.

### Architecture of ROS 2

ROS 2 uses a distributed architecture based on DDS (Data Distribution Service), which is a specification from the Object Management Group (OMG) that defines a data-centric communications middleware. This architecture provides several advantages over ROS 1:

1. **Real-time support**: Better support for real-time systems
2. **Multi-robot systems**: Easier to create and manage multi-robot systems
3. **Security**: Built-in security features
4. **Reliability**: More robust communication between nodes
5. **OS/platform independence**: Better support for different operating systems and platforms

### Key Components of ROS 2

#### Nodes
A node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 application. Multiple nodes can be distributed across multiple devices and communicate with each other through topics, services, and actions.

#### Topics and Messages
Topics are named buses over which nodes exchange messages. Messages are the data that flows between nodes. The ROS 2 message system is typed and can be serialized for transmission between nodes.

#### Services
Services provide a request/reply communication pattern. A node sends a request to another node and waits for a response.

#### Actions
Actions are used for long-running tasks. They provide feedback during execution and can be preempted.

#### Parameters
Parameters are named, typed values that can be set and retrieved by nodes. They allow for configuration of nodes at runtime.

#### Launch Files
Launch files allow you to start multiple nodes with a single command, along with their configurations.

## Implementation

Here's an example of a simple ROS 2 publisher node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

And here's a corresponding subscriber:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

1. **Logical Exercise**: Explain the advantages of ROS 2 over ROS 1, particularly in terms of real-time support and multi-robot systems.

2. **Conceptual Exercise**: Describe the role of DDS in ROS 2 architecture and how it differs from the ROS 1 communication model.

3. **Implementation Exercise**: Create a ROS 2 package with a publisher and subscriber node. Configure the publisher to publish sensor data (e.g., temperature readings) and the subscriber to log this data.

## Summary

This chapter introduced the ROS 2 framework, its architecture based on DDS, and basic concepts including nodes, topics, services, and actions. We've seen how ROS 2 addresses limitations in ROS 1 through its more robust and scalable architecture. In the following chapters, we'll explore these concepts in more depth and build more complex robotic applications.

## References
- ROS 2 Documentation: https://docs.ros.org/
- DDS Standard: https://www.omg.org/spec/DDS/
- ROS 2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html