---
title: Chapter 2 - Nodes, Topics, Services, and Actions
description: Detailed exploration of ROS 2 communication patterns
keywords: [ros2, nodes, topics, services, actions, communication]
sidebar_position: 3
module_ref: module-1-ros2
prerequisites: ["Module 1, Chapter 1"]
learning_objectives: ["Implement ROS 2 nodes in Python and C++", "Understand communication patterns", "Create publishers, subscribers, services, and actions"]
estimated_reading_time: 45
exercises_count: 3
---

# Chapter 2: Nodes, Topics, Services, and Actions

## Learning Objectives
- Implement ROS 2 nodes in both Python and C++
- Understand the different communication patterns in ROS 2
- Create publishers, subscribers, services, and actions
- Debug communication issues in ROS 2 systems

## Prerequisites
- Understanding of ROS 2 architecture from Chapter 1
- Basic programming skills in Python and C++
- ROS 2 development environment setup

## Core Concepts

ROS 2 provides several communication patterns that enable nodes to exchange information:

### Nodes
Nodes are the fundamental building blocks of ROS 2 applications. Each node is a process that performs computation and can communicate with other nodes. Nodes in ROS 2 are more robust than in ROS 1, with better lifecycle management and introspection capabilities.

### Topics and Messages (Publish/Subscribe)
Topics provide asynchronous communication between nodes using a publish/subscribe model. Publishers send messages to topics, and subscribers receive messages from topics. This decouples publishers from subscribers, allowing for flexible system design.

### Services (Request/Reply)
Services provide synchronous communication using a request/reply model. A client sends a request to a service and waits for a response. This is useful for operations that require a direct response.

### Actions (Long-Running Tasks)
Actions are designed for long-running tasks that require feedback during execution. They allow clients to send goals to action servers, receive feedback during execution, and get results when complete. Actions also support preempting goals.

## Implementation

### Creating a Node in Python
Here's a more complex example that combines topics, services, and actions:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class ComplexNode(Node):
    def __init__(self):
        super().__init__('complex_node')
        
        # Create a publisher
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        
        # Create a service server
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        
        # Create an action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        
        # Create a timer to publish messages
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {request.a} + {request.b} = {response.sum}')
        return response

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info(f'Publishing feedback: {feedback_msg.sequence}')
            
            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Returning result: {result.sequence}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ComplexNode()
    
    # Use a multi-threaded executor to handle all callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Node in C++
For comparison, here's a similar node implemented in C++:

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using AddTwoInts = example_interfaces::srv::AddTwoInts;
using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class ComplexNode : public rclcpp::Node
{
public:
    ComplexNode()
    : Node("complex_node_cpp")
    {
        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        
        // Create service
        service_ = this->create_service<AddTwoInts>(
            "add_two_ints",
            std::bind(&ComplexNode::add_two_ints_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        // Create action server
        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&ComplexNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ComplexNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&ComplexNode::handle_accepted, this, std::placeholders::_1));
        
        // Create timer
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&ComplexNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello World " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    
    void add_two_ints_callback(const AddTwoInts::Request::SharedPtr request,
                              const AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld, b: %ld", request->a, request->b);
        RCLCPP_INFO(this->get_logger(), "Sending response: [%ld]", response->sum);
    }
    
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&ComplexNode::execute, this, _1), goal_handle}.detach();
    }
    
    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        rclcpp::Rate rate(1);
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto result = std::make_shared<Fibonacci::Result>();

        auto sequence = std::vector<int64_t>();
        sequence.push_back(0);
        sequence.push_back(1);

        auto goal = goal_handle->get_goal();
        for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
            if (goal_handle->is_canceling()) {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal Canceled");
                return;
            }
            
            sequence.push_back(sequence[i] + sequence[i - 1]);
            feedback->sequence = sequence;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish Feedback");
            
            rate.sleep();
        }

        if (rclcpp::ok()) {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
        }
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Service<AddTwoInts>::SharedPtr service_;
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ComplexNode>());
    rclcpp::shutdown();
    return 0;
}
```

## Quality of Service (QoS) Settings

ROS 2 includes Quality of Service (QoS) settings that allow you to specify the reliability and durability of communication:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Create a QoS profile with specific settings
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,  # or BEST_EFFORT
    durability=DurabilityPolicy.VOLATILE,    # or TRANSIENT_LOCAL
)
```

## Exercises

1. **Logical Exercise**: Analyze when to use topics vs services vs actions. Provide specific examples of use cases for each communication pattern.

2. **Conceptual Exercise**: Explain Quality of Service (QoS) settings and their importance in robot communication. How do reliability and durability policies affect system behavior?

3. **Implementation Exercise**: Create a ROS 2 package that includes a publisher, subscriber, service server/client, and action server/client. The system should simulate a simple robot that receives navigation goals, provides feedback during navigation, and reports results when complete.

## Summary

This chapter explored the fundamental communication patterns in ROS 2: topics for publish/subscribe communication, services for request/reply interactions, and actions for long-running tasks with feedback. We've implemented examples in both Python and C++ and discussed Quality of Service settings that control the reliability and durability of communication. These communication patterns form the backbone of most ROS 2 applications and are essential for building distributed robotic systems.

## References
- [ROS 2 Communication Patterns](https://docs.ros.org/en/humble/Concepts/About-Topics-Services-Actions.html)
- [Quality of Service Settings](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-settings.html)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)