# API Contracts: Module 2: Sensors & Actuators

## Overview
This document defines the conceptual API contracts and interfaces for the Sensors & Actuators learning module. These represent the ROS 2 interfaces for sensor reading and actuator control that students will learn to use and implement.

## Sensor Data Interface Specifications

### IMU Sensor Interface
```python
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion

class IMUInterface:
    """
    Interface for IMU sensor data handling
    """
    def get_orientation(self) -> Quaternion:
        """Get orientation as quaternion (x, y, z, w)"""
        pass

    def get_angular_velocity(self) -> Vector3:
        """Get angular velocity (x, y, z) in rad/s"""
        pass

    def get_linear_acceleration(self) -> Vector3:
        """Get linear acceleration (x, y, z) in m/s²"""
        pass

    def get_orientation_euler(self) -> tuple[float, float, float]:
        """Get orientation as Euler angles (roll, pitch, yaw) in radians"""
        pass
```

### LIDAR Sensor Interface
```python
from sensor_msgs.msg import LaserScan

class LIDARInterface:
    """
    Interface for LIDAR sensor data handling
    """
    def get_ranges(self) -> list[float]:
        """Get distance measurements for each angle"""
        pass

    def get_min_distance(self) -> float:
        """Get minimum distance measurement"""
        pass

    def get_max_distance(self) -> float:
        """Get maximum distance measurement"""
        pass

    def get_distance_at_angle(self, angle: float) -> float:
        """Get distance measurement at a specific angle (in radians)"""
        pass

    def get_field_of_view(self) -> tuple[float, float]:
        """Get field of view (min_angle, max_angle) in radians"""
        pass
```

### Camera Sensor Interface
```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraInterface:
    """
    Interface for camera sensor data handling
    """
    def get_image(self) -> Image:
        """Get raw image message"""
        pass

    def get_cv_image(self) -> object:  # OpenCV image object
        """Get image as OpenCV format"""
        pass

    def get_resolution(self) -> tuple[int, int]:
        """Get image resolution (width, height)"""
        pass

    def detect_edges(self, image=None) -> object:
        """Apply edge detection to image"""
        pass

    def detect_objects(self, image=None) -> list:
        """Detect objects in image and return bounding boxes"""
        pass
```

## Actuator Control Interface Specifications

### Servo Actuator Interface
```python
from std_msgs.msg import Float64

class ServoInterface:
    """
    Interface for servo actuator control
    """
    def set_position(self, position: float):
        """Set servo position in radians"""
        pass

    def get_position(self) -> float:
        """Get current servo position in radians"""
        pass

    def set_velocity(self, velocity: float):
        """Set servo velocity in rad/s (if supported)"""
        pass

    def get_range(self) -> tuple[float, float]:
        """Get position range (min, max) in radians"""
        pass
```

### Motor Actuator Interface
```python
from std_msgs.msg import Float64

class MotorInterface:
    """
    Interface for motor actuator control
    """
    def set_velocity(self, velocity: float):
        """Set motor velocity in rad/s"""
        pass

    def set_torque(self, torque: float):
        """Set motor torque in Nm"""
        pass

    def get_velocity(self) -> float:
        """Get current motor velocity in rad/s"""
        pass

    def get_torque(self) -> float:
        """Get current motor torque in Nm"""
        pass
```

## AI Decision Interface Specification

### Sensor-Based Decision Interface
```python
from sensor_msgs.msg import Imu, LaserScan, Image
from std_msgs.msg import Float64

class AIDecisionInterface:
    """
    Interface for AI-based decision making using sensor data
    """
    def process_sensor_data(self, sensor_data) -> dict:
        """Process sensor data and extract relevant information"""
        pass

    def make_decision(self, processed_data: dict) -> dict:
        """Make decision based on processed sensor data"""
        pass

    def generate_actuator_commands(self, decision: dict) -> list:
        """Generate actuator commands from decision"""
        pass

    def execute_decision_loop(self):
        """Main loop: read sensors → process → decide → actuate"""
        pass
```

## ROS 2 Node Interface Specifications

### Sensor Reading Node Interface
```python
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, Image

class SensorReadingNodeInterface(Node):
    """
    Interface for sensor reading nodes
    """
    def __init__(self, node_name: str, sensor_topic: str, sensor_type: str):
        super().__init__(node_name)

    def subscribe_to_sensor(self, topic: str, sensor_type):
        """Subscribe to sensor topic"""
        pass

    def process_sensor_message(self, msg):
        """Process incoming sensor message"""
        pass

    def publish_processed_data(self, data):
        """Publish processed sensor data to other nodes"""
        pass
```

### Actuator Control Node Interface
```python
from rclpy.node import Node
from std_msgs.msg import Float64

class ActuatorControlNodeInterface(Node):
    """
    Interface for actuator control nodes
    """
    def __init__(self, node_name: str, actuator_topic: str):
        super().__init__(node_name)

    def publish_actuator_command(self, command):
        """Publish command to actuator"""
        pass

    def validate_command(self, command) -> bool:
        """Validate actuator command before publishing"""
        pass

    def get_actuator_status(self) -> dict:
        """Get current actuator status"""
        pass
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
  "sensorTypes": ["enum (IMU|LIDAR|CAMERA|OTHER)"],
  "actuatorTypes": ["enum (SERVO|MOTOR|OTHER)"],
  "aiIntegration": "boolean",
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (DRAFT|REVIEW|PUBLISHED)"
}
```

### Sensor Example Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "name": "string",
  "type": "enum (IMU|LIDAR|CAMERA|SONAR|GPS|OTHER)",
  "description": "string",
  "rosMessageType": "string",
  "exampleCode": "string (Python code)",
  "simulationConfig": "object",
  "learningObjectives": ["string"],
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (ACTIVE|ARCHIVED)"
}
```

### Actuator Example Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "name": "string",
  "type": "enum (SERVO|MOTOR|LINEAR_ACTUATOR|GRIPPER|OTHER)",
  "description": "string",
  "controlType": "enum (POSITION|VELOCITY|TORQUE|PWM)",
  "exampleCode": "string (Python code)",
  "simulationConfig": "object",
  "learningObjectives": ["string"],
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (ACTIVE|ARCHIVED)"
}
```

## Error Handling Patterns

### Sensor Error Interface
```python
class SensorError(Exception):
    """Base exception for sensor-related errors"""
    def __init__(self, sensor_type: str, message: str, error_code: str = None):
        self.sensor_type = sensor_type
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)

# Example usage:
try:
    # Sensor operation
    pass
except SensorError as e:
    node.get_logger().error(f'Sensor Error ({e.sensor_type}): {e.message} (Code: {e.error_code})')
```

### Actuator Error Interface
```python
class ActuatorError(Exception):
    """Base exception for actuator-related errors"""
    def __init__(self, actuator_type: str, message: str, error_code: str = None):
        self.actuator_type = actuator_type
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)

# Example usage:
try:
    # Actuator operation
    pass
except ActuatorError as e:
    node.get_logger().error(f'Actuator Error ({e.actuator_type}): {e.message} (Code: {e.error_code})')
```

## Validation Interfaces

### Sensor Data Validation Interface
```python
class SensorDataValidatorInterface:
    """
    Interface for validating sensor data
    """
    def validate_imu_data(self, imu_msg) -> dict:
        """Validate IMU message and return validation report"""
        pass

    def validate_lidar_data(self, lidar_msg) -> dict:
        """Validate LIDAR message and return validation report"""
        pass

    def validate_camera_data(self, camera_msg) -> dict:
        """Validate camera message and return validation report"""
        pass

    def check_sensor_fusion(self, sensor_data_list: list) -> dict:
        """Validate consistency across multiple sensors"""
        pass
```

### Actuator Command Validation Interface
```python
class ActuatorCommandValidatorInterface:
    """
    Interface for validating actuator commands
    """
    def validate_servo_command(self, position: float) -> bool:
        """Validate servo position command"""
        pass

    def validate_motor_command(self, velocity: float) -> bool:
        """Validate motor velocity command"""
        pass

    def check_safety_limits(self, command: dict) -> dict:
        """Check if command violates safety limits"""
        pass

    def validate_command_syntax(self, command) -> dict:
        """Validate command syntax and return errors if any"""
        pass
```