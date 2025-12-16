
# ROS 2 Fundamentals 2: Python Agents & rclpy

:::info Learning Objectives
* Master the rclpy library for Python-based ROS 2 development
* Create Python nodes that interact with ROS 2 topics and services
* Implement action servers and clients in Python
* Understand asynchronous programming patterns in ROS 2 Python nodes
:::
![Hero Diagram: Python Agents & rclpy](../../static/img/ros2-network.png)

The `rclpy` library provides Python bindings for ROS 2, enabling developers to create ROS 2 nodes using Python. Python's simplicity and rich ecosystem make it an excellent choice for rapid prototyping and development of robotic applications.

## Getting Started with rclpy

The `rclpy` library serves as the Python client library for ROS 2, providing the interface between Python applications and the ROS 2 middleware. To begin using `rclpy`, you need to initialize the library and create a node.

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

:::tip Key Concept
The `rclpy.init()` function initializes the ROS 2 client library, and `rclpy.spin()` keeps the node active, processing callbacks for subscriptions, services, and timers until the node is shut down.
:::

## Creating Publishers and Subscribers

Working with topics in Python using rclpy involves creating publishers and subscribers within your node class:

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
    minimal_publisher = MinimalPublisher()
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

## Working with Services in Python

Services in ROS 2 provide synchronous request-response communication. Here's how to implement a service server and client in Python:

```python
# Service Server
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

## Asynchronous Programming Patterns

ROS 2 Python nodes can leverage asyncio for handling multiple operations concurrently:

```python
import rclpy
from rclpy.node import Node
import asyncio

class AsyncNode(Node):
    def __init__(self):
        super().__init__('async_node')
        # Using asyncio for concurrent operations
        self.future = asyncio.Future()
```

## Hands-on Lab

Create a Python-based ROS 2 system that:
1. Implements a publisher that sends sensor data
2. Creates a subscriber that processes this data
3. Develops a service server that performs calculations
4. Builds a client that calls the service
5. Uses asynchronous patterns for concurrent operations

## Self-Assessment

1. What is rclpy and why is it important for Python ROS 2 development?
2. How do you create a publisher and subscriber in Python using rclpy?
3. What are the advantages of using Python for ROS 2 development?