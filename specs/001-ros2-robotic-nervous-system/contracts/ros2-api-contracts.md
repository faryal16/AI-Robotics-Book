# API Contracts: Module 1: The Robotic Nervous System (ROS 2)

## Overview
This document defines the conceptual API contracts and interfaces for the ROS 2 learning module. These represent the ROS 2 interfaces that students will learn to use and implement.

## ROS 2 Node Interface Specification

### Base Node Interface
```python
from rclpy.node import Node
from std_msgs.msg import String

class BaseRobotNode(Node):
    """
    Base interface for all robot nodes in the educational examples
    """
    def __init__(self, node_name: str):
        super().__init__(node_name)

    def get_node_name(self) -> str:
        """Returns the name of the node"""
        pass

    def get_logger(self):
        """Returns the node's logger"""
        pass
```

### Publisher Interface
```python
from rclpy.publisher import Publisher

class PublisherInterface:
    """
    Interface for publisher nodes
    """
    def create_publisher(self, msg_type, topic_name: str, qos_profile) -> Publisher:
        """Create a publisher for a specific message type and topic"""
        pass

    def publish(self, msg):
        """Publish a message to the topic"""
        pass
```

### Subscriber Interface
```python
from rclpy.subscription import Subscription

class SubscriberInterface:
    """
    Interface for subscriber nodes
    """
    def create_subscription(self, msg_type, topic_name: str, callback, qos_profile) -> Subscription:
        """Create a subscription to a specific topic"""
        pass

    def get_subscription_topics(self) -> list:
        """Get list of topics this node subscribes to"""
        pass
```

## ROS 2 Service Interface Specification

### Service Server Interface
```python
from rclpy.service import Service

class ServiceServerInterface:
    """
    Interface for service server nodes
    """
    def create_service(self, srv_type, srv_name: str, callback) -> Service:
        """Create a service server"""
        pass

    def get_service_name(self) -> str:
        """Get the name of the service"""
        pass
```

### Service Client Interface
```python
from rclpy.client import Client

class ServiceClientInterface:
    """
    Interface for service client nodes
    """
    def create_client(self, srv_type, srv_name: str) -> Client:
        """Create a service client"""
        pass

    def call_async(self, request):
        """Make an asynchronous service call"""
        pass

    def wait_for_service(self, timeout_sec: float) -> bool:
        """Wait for service to become available"""
        pass
```

## URDF Model Interface Specification

### URDF Link Definition
```xml
<link name="string">
  <inertial>
    <mass value="float"/>
    <origin xyz="float float float" rpy="float float float"/>
    <inertia ixx="float" ixy="float" ixz="float" iyy="float" iyz="float" izz="float"/>
  </inertial>
  <visual>
    <origin xyz="float float float" rpy="float float float"/>
    <geometry>
      <!-- geometry definition -->
    </geometry>
    <material name="string"/>
  </visual>
  <collision>
    <origin xyz="float float float" rpy="float float float"/>
    <geometry>
      <!-- geometry definition -->
    </geometry>
  </collision>
</link>
```

### URDF Joint Definition
```xml
<joint name="string" type="string">
  <parent link="string"/>
  <child link="string"/>
  <origin xyz="float float float" rpy="float float float"/>
  <axis xyz="float float float"/>
  <limit lower="float" upper="float" effort="float" velocity="float"/>
</joint>
```

## Educational Content API

### Tutorial Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "title": "string",
  "description": "string",
  "files": ["string"],
  "dependencies": ["string"],
  "instructions": "string (Markdown format)",
  "expectedOutcome": "string",
  "prerequisites": ["string"],
  "duration": "integer (minutes)",
  "difficulty": "enum (BEGINNER|INTERMEDIATE|ADVANCED)",
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (DRAFT|REVIEW|PUBLISHED)"
}
```

### Learning Objective Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "title": "string",
  "description": "string",
  "achievementCriteria": ["string"],
  "relatedTopics": ["string"],
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (ACTIVE|ARCHIVED)"
}
```

## Error Handling Patterns

### Standard Error Response Pattern for ROS 2 Nodes
```python
class ROS2NodeError(Exception):
    """Base exception for ROS 2 node errors"""
    def __init__(self, message: str, error_code: str = None):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)

# Example usage in nodes:
try:
    # ROS 2 operation
    pass
except ROS2NodeError as e:
    node.get_logger().error(f'ROS 2 Error: {e.message} (Code: {e.error_code})')
```

## Validation Interfaces

### URDF Validation Interface
```python
class URDFValidatorInterface:
    """
    Interface for URDF validation
    """
    def validate_syntax(self, urdf_content: str) -> bool:
        """Validate URDF XML syntax"""
        pass

    def validate_robot_model(self, urdf_file_path: str) -> dict:
        """Validate robot model semantics and return validation report"""
        pass

    def check_joint_limits(self, urdf_file_path: str) -> list:
        """Check for invalid joint limit definitions"""
        pass
```

### Code Example Validation Interface
```python
class CodeExampleValidatorInterface:
    """
    Interface for validating code examples
    """
    def validate_python_syntax(self, code: str) -> dict:
        """Validate Python syntax and return errors if any"""
        pass

    def validate_ros2_patterns(self, code: str) -> list:
        """Validate ROS 2 coding patterns and best practices"""
        pass

    def check_dependencies(self, code: str) -> list:
        """Check for missing ROS 2 dependencies"""
        pass
```