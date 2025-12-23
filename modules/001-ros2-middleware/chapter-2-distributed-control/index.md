# Chapter 2: Distributed Control Patterns

## Learning Objectives

After completing this chapter, you will be able to:
- Implement service-based request/response communication
- Create and use action-based goal-oriented communication
- Design distributed control architectures
- Handle errors and exceptions in distributed systems

## 2.1 Services in ROS 2

Services provide synchronous request/response communication between nodes. This pattern is useful for operations that have a clear beginning and end.

### Creating a Service Server

```python
# add_two_ints_server.py
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response


def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Creating a Service Client

```python
# add_two_ints_client.py
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (1, 2, response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 2.2 Actions in ROS 2

Actions are designed for long-running tasks that require feedback and the ability to cancel. They're perfect for robot navigation and manipulation tasks.

### Creating an Action Server

```python
# fibonacci_action_server.py
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer
from rclpy.node import Node
import rclpy


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result


def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 2.3 Distributed Control Architecture

In humanoid robotics, distributed control is essential for real-time performance and fault tolerance. Here's a pattern for organizing nodes:

### Joint Controller Node
- Controls a specific joint or group of joints
- Receives commands via topics or services
- Publishes joint state feedback

### Sensor Processing Node
- Reads data from sensors
- Processes and filters sensor data
- Publishes processed information

### High-Level Planner Node
- Makes decisions based on processed information
- Sends commands to controllers
- Coordinates between different subsystems

## 2.4 Error Handling in Distributed Systems

Proper error handling is critical in distributed robotics systems:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class RobustNode(Node):

    def __init__(self):
        super().__init__('robust_node')

        # Configure QoS for reliable communication
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        )

        self.publisher = self.create_publisher(String, 'robust_topic', qos_profile)
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.safe_callback,
            qos_profile
        )

        # Timer for periodic health checks
        self.timer = self.create_timer(1.0, self.health_check)

    def safe_callback(self, msg):
        try:
            # Process the message
            processed_data = self.process_message(msg)
            self.publisher.publish(processed_data)
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')
            # Implement fallback behavior
            self.fallback_behavior()

    def health_check(self):
        # Perform system health checks
        if not self.system_healthy():
            self.get_logger().warning('System health check failed')
            # Implement recovery procedures
            self.recovery_procedure()
```

## Exercises

1. Create a distributed control system for a 6-DOF robotic arm using services for inverse kinematics and actions for trajectory execution
2. Implement a fault-tolerant sensor fusion node that can continue operating when individual sensors fail
3. Design a recovery procedure for when communication between nodes is lost

## Summary

This chapter covered advanced ROS 2 communication patterns including services and actions. You've learned how to design distributed control architectures for humanoid robots and implement proper error handling for robust operation.