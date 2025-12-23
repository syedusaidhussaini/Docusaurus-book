# Chapter 3: Advanced ROS 2 Patterns

## Learning Objectives

After completing this chapter, you will be able to:
- Configure and manage ROS 2 parameters effectively
- Implement lifecycle nodes for complex system management
- Design robust communication patterns with custom QoS settings
- Create and use custom message types for humanoid robotics

## 3.1 Parameter Management

Parameters in ROS 2 allow for runtime configuration of nodes without recompilation. This is crucial for humanoid robots that need to adapt to different environments or tasks.

### Basic Parameter Declaration and Usage

```python
# parameter_node.py
import rclpy
from rclpy.node import Node


class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values and descriptions
        self.declare_parameter('control_frequency', 50,
                             'Frequency for control loop in Hz')
        self.declare_parameter('max_velocity', 1.0,
                             'Maximum joint velocity in rad/s')
        self.declare_parameter('safety_margin', 0.1,
                             'Safety margin for joint limits')

        # Get parameter values
        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_margin = self.get_parameter('safety_margin').value

        # Create timer that uses the parameter
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop
        )

        # Set parameter callback to handle runtime changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'control_frequency' and param.type_ == Parameter.Type.INTEGER:
                self.control_frequency = param.value
                # Adjust timer period based on new frequency
                self.timer.timer_period_ns = int(1e9 / self.control_frequency)
            elif param.name == 'max_velocity' and param.type_ == Parameter.Type.DOUBLE:
                self.max_velocity = param.value
        return SetParametersResult(successful=True)

    def control_loop(self):
        # Implement control logic using parameters
        self.get_logger().info(f'Control loop running at {self.control_frequency}Hz')
```

## 3.2 Lifecycle Nodes

Lifecycle nodes provide a structured way to manage the state of complex robotic components:

```python
# lifecycle_controller.py
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
import rclpy


class JointController(LifecycleNode):

    def __init__(self):
        super().__init__('joint_controller')
        self.current_joint_position = 0.0

    def on_configure(self, state: LifecycleState):
        self.get_logger().info(f'Configuring {self.get_name()}')

        # Initialize resources
        self.position_publisher = self.create_publisher(
            JointState, 'joint_states', 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info(f'Activating {self.get_name()}')

        # Activate publishers/subscribers
        self.position_publisher.on_activate()

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info(f'Deactivating {self.get_name()}')

        # Deactivate publishers/subscribers
        self.position_publisher.on_deactivate()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info(f'Cleaning up {self.get_name()}')

        # Clean up resources
        self.position_publisher.destroy()

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info(f'Shutting down {self.get_name()}')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState):
        self.get_logger().info(f'Error in {self.get_name()}')
        return TransitionCallbackReturn.SUCCESS
```

## 3.3 Custom Message Types for Humanoid Robotics

Creating custom message types for humanoid-specific data:

### Define the message (in msg/HumanoidState.msg):
```
# Humanoid robot state message
Header header
float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts
geometry_msgs/Pose[] link_poses
sensor_msgs/Imu imu_data
bool in_motion
bool emergency_stop
```

### Using custom messages in nodes:

```python
# humanoid_state_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from your_custom_msgs.msg import HumanoidState  # Your custom message
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu


class HumanoidStatePublisher(Node):

    def __init__(self):
        super().__init__('humanoid_state_publisher')
        self.publisher = self.create_publisher(
            HumanoidState,
            'humanoid_state',
            10
        )

        # Initialize robot state
        self.joint_names = [
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle',
            'left_shoulder', 'left_elbow', 'left_wrist',
            'right_shoulder', 'right_elbow', 'right_wrist'
        ]

        self.state_timer = self.create_timer(0.02, self.publish_state)  # 50Hz

    def publish_state(self):
        msg = HumanoidState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Populate joint states
        msg.joint_positions = self.get_current_joint_positions()
        msg.joint_velocities = self.get_current_joint_velocities()
        msg.joint_efforts = self.get_current_joint_efforts()

        # Populate link poses
        msg.link_poses = self.get_link_poses()

        # Populate IMU data
        msg.imu_data = self.get_imu_data()

        # Set status flags
        msg.in_motion = self.is_robot_moving()
        msg.emergency_stop = self.emergency_stop_active()

        self.publisher.publish(msg)

    def get_current_joint_positions(self):
        # Implementation to get current joint positions
        return [0.0] * len(self.joint_names)

    def get_current_joint_velocities(self):
        # Implementation to get current joint velocities
        return [0.0] * len(self.joint_names)

    def get_current_joint_efforts(self):
        # Implementation to get current joint efforts
        return [0.0] * len(self.joint_names)
```

## 3.4 Advanced QoS Patterns

Different QoS settings for different types of data in humanoid robotics:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class HumanoidNode(Node):

    def __init__(self):
        super().__init__('humanoid_node')

        # High-frequency sensor data (e.g., IMU) - best effort, volatile
        sensor_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )

        # Critical control commands - reliable, transient local
        control_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        )

        # Configuration parameters - reliable, durable
        config_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        )

        # Create publishers with appropriate QoS
        self.sensor_pub = self.create_publisher(SensorMsg, 'sensors', sensor_qos)
        self.control_pub = self.create_publisher(ControlMsg, 'commands', control_qos)
        self.config_pub = self.create_publisher(ConfigMsg, 'config', config_qos)
```

## 3.5 System Integration Patterns

Pattern for integrating multiple subsystems in a humanoid robot:

```python
class HumanoidController(Node):

    def __init__(self):
        super().__init__('humanoid_controller')

        # Subsystem controllers
        self.upper_body_controller = UpperBodyController()
        self.lower_body_controller = LowerBodyController()
        self.safety_controller = SafetyController()

        # State estimation
        self.state_estimator = StateEstimator()

        # Main control timer
        self.control_timer = self.create_timer(0.02, self.main_control_loop)  # 50Hz

        # Emergency stop publisher
        self.emergency_stop_pub = self.create_publisher(Bool, 'emergency_stop', 1)

    def main_control_loop(self):
        # Update robot state
        current_state = self.state_estimator.get_state()

        # Check safety conditions
        if self.safety_controller.is_safe(current_state):
            # Execute planned motions
            upper_body_cmd = self.upper_body_controller.compute_commands(current_state)
            lower_body_cmd = self.lower_body_controller.compute_commands(current_state)

            # Send commands to hardware
            self.send_commands(upper_body_cmd, lower_body_cmd)
        else:
            # Trigger safety procedures
            self.trigger_safety_procedures()

    def trigger_safety_procedures(self):
        # Publish emergency stop
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)

        # Stop all motion
        self.upper_body_controller.stop()
        self.lower_body_controller.stop()
```

## Exercises

1. Implement a parameter server that manages all humanoid robot parameters in one place
2. Create a custom message type for humanoid walking patterns and implement a gait generator
3. Design a fault-tolerant system that can continue operating when one subsystem fails

## Summary

This chapter covered advanced ROS 2 patterns essential for humanoid robotics, including parameter management, lifecycle nodes, custom message types, and advanced QoS configurations. You've learned how to design robust, integrated systems for complex humanoid robots.