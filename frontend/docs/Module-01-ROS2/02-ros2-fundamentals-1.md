
# ROS 2 Fundamentals 1: Nodes, Topics, and Services

:::info Learning Objectives
* Understand the core concepts of ROS 2 architecture
* Learn how to create and manage ROS 2 nodes
* Explore the publish-subscribe communication model using topics
* Master service-based communication patterns
:::
![Hero Diagram: ROS 2 Nodes, Topics, Services](../../static/img/ros2-network2.jpg)

The Robot Operating System 2 (ROS 2) provides a flexible framework for developing robot applications. At its core, ROS 2 is designed around a **distributed** architecture that enables communication between different software components called *nodes*.

## Understanding ROS 2 Nodes

A **node** is a fundamental component of a ROS 2 system that performs computation. Nodes are typically organized to perform specific tasks, such as sensor processing, motion planning, or control. In ROS 2, nodes are implemented as processes that communicate with other nodes through various mechanisms.

### Creating Your First Node

To create a node in ROS 2, you typically inherit from the `rclcpp::Node` class (for C++) or use the `rclpy` library (for Python). Each node must have a unique name within the ROS 2 domain to prevent conflicts.

```bash
# Create a new ROS 2 package for our node
ros2 pkg create --build-type ament_python my_robot_node
```

## Communication Patterns: Topics

ROS 2 uses a **publish-subscribe** model for asynchronous communication through *topics*. Publishers send messages to a topic, and subscribers receive messages from that topic. This decouples the sender and receiver in both time and space.

:::tip Key Concept
Topics in ROS 2 use a publish-subscribe communication model where multiple publishers can send to the same topic and multiple subscribers can receive from the same topic, enabling flexible system architectures.
:::

### Working with Topics

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

## Communication Patterns: Services

Services provide a **request-response** communication model where a client sends a request to a server and waits for a response. This synchronous communication pattern is useful for operations that require a guaranteed response.

### Implementing Services

```bash
# Create a service definition
# example_srv/srv/AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

Services are ideal for operations that have a clear input/output relationship and where the client needs to wait for completion.

## Hands-on Lab

Create a simple ROS 2 system with:
1. A publisher node that publishes sensor data
2. A subscriber node that processes the data
3. A service server that performs calculations
4. A service client that requests those calculations

## Self-Assessment

1. What is the difference between topics and services in ROS 2?
2. Why is the publish-subscribe model beneficial for robotics applications?
3. How do nodes communicate in a ROS 2 system?