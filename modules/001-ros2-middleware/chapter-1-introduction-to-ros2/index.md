# Chapter 1: Introduction to ROS 2

## Learning Objectives

After completing this chapter, you will be able to:
- Understand the fundamental concepts of ROS 2
- Create and run basic ROS 2 nodes
- Implement publisher/subscriber communication
- Use ROS 2 command-line tools effectively

## 1.1 ROS 2 Architecture Overview

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Key Concepts

- **Nodes**: Processes that perform computation
- **Topics**: Named buses over which nodes exchange messages
- **Messages**: ROS data types used when publishing or subscribing to a topic
- **Services**: Synchronous request/response communication
- **Actions**: Asynchronous goal-oriented communication

## 1.2 Setting Up Your First ROS 2 Node

Let's create a simple publisher node that broadcasts a message:

```python
# publisher_member_function.py
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

## 1.3 Creating a Subscriber Node

Now, let's create a subscriber that listens to the publisher:

```python
# subscriber_member_function.py
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

## 1.4 Running the Example

1. Source your ROS 2 installation:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Build your package:
   ```bash
   colcon build --packages-select your_package_name
   ```

3. Source the local setup:
   ```bash
   source install/setup.bash
   ```

4. Run the publisher in one terminal:
   ```bash
   ros2 run your_package_name publisher_member_function
   ```

5. Run the subscriber in another terminal:
   ```bash
   ros2 run your_package_name subscriber_member_function
   ```

## 1.5 Quality of Service (QoS) Settings

QoS settings are crucial for real-time robotics applications:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Create a QoS profile for real-time control
qos_profile = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST
)
```

## Exercises

1. Modify the publisher to send robot joint positions instead of "Hello World" messages
2. Create a new node that subscribes to multiple topics simultaneously
3. Experiment with different QoS settings and observe their effects on communication

## Summary

This chapter introduced the fundamental concepts of ROS 2, including nodes, topics, and basic publisher/subscriber patterns. You've learned how to create simple ROS 2 nodes and run them in a distributed system.