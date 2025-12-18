# Quickstart: Module 2: Sensors & Actuators

## Prerequisites

- Completed Module 1: The Robotic Nervous System (ROS 2)
- Ubuntu 22.04 LTS with ROS 2 Humble installed
- Python 3.10+
- Basic understanding of ROS 2 concepts (Nodes, Topics, Services)
- Git for version control

## Setup Environment

### 1. Verify ROS 2 Installation
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Verify installation
ros2 topic list
```

### 2. Install Additional Dependencies
```bash
# Install sensor-related packages
sudo apt update
sudo apt install ros-humble-vision-opencv ros-humble-cv-bridge ros-humble-image-transport
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
sudo apt install python3-opencv python3-matplotlib
```

## Creating Sensor Reading Examples

### 1. Create a Sensor Reading Package
```bash
mkdir -p ~/ros2_ws/src/sensor_examples
cd ~/ros2_ws/src/sensor_examples
ros2 pkg create --build-type ament_python sensor_reader --dependencies rclpy sensor_msgs geometry_msgs std_msgs cv_bridge
```

### 2. IMU Reader Node
Create `~/ros2_ws/src/sensor_examples/sensor_reader/imu_reader.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class ImuReader(Node):
    def __init__(self):
        super().__init__('imu_reader')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Timer to simulate IMU data if no real sensor available
        self.timer = self.create_timer(0.1, self.publish_mock_data)
        self.publisher = self.create_publisher(Imu, '/mock_imu/data', 10)

    def imu_callback(self, msg):
        # Extract orientation (in quaternion)
        orientation_q = msg.orientation
        # Convert to Euler angles for easier understanding
        euler = self.quaternion_to_euler(orientation_q)

        # Extract angular velocity
        angular_velocity = msg.angular_velocity

        # Extract linear acceleration
        linear_accel = msg.linear_acceleration

        self.get_logger().info(
            f'Orientation: Roll={euler[0]:.2f}, Pitch={euler[1]:.2f}, Yaw={euler[2]:.2f}\n'
            f'Angular Vel: X={angular_velocity.x:.2f}, Y={angular_velocity.y:.2f}, Z={angular_velocity.z:.2f}\n'
            f'Linear Acc: X={linear_accel.x:.2f}, Y={linear_accel.y:.2f}, Z={linear_accel.z:.2f}'
        )

    def quaternion_to_euler(self, quaternion):
        import math
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]

    def publish_mock_data(self):
        import random
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Mock orientation (in radians)
        msg.orientation.x = random.uniform(-0.1, 0.1)
        msg.orientation.y = random.uniform(-0.1, 0.1)
        msg.orientation.z = random.uniform(-0.1, 0.1)
        msg.orientation.w = 1.0  # Normalize

        # Mock angular velocity (in rad/s)
        msg.angular_velocity.x = random.uniform(-0.01, 0.01)
        msg.angular_velocity.y = random.uniform(-0.01, 0.01)
        msg.angular_velocity.z = random.uniform(-0.01, 0.01)

        # Mock linear acceleration (in m/s^2)
        msg.linear_acceleration.x = random.uniform(-0.1, 0.1)
        msg.linear_acceleration.y = random.uniform(-0.1, 0.1)
        msg.linear_acceleration.z = 9.81 + random.uniform(-0.1, 0.1)  # Gravity

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_reader = ImuReader()
    rclpy.spin(imu_reader)
    imu_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. LIDAR Reader Node
Create `~/ros2_ws/src/sensor_examples/sensor_reader/lidar_reader.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Timer to simulate LIDAR data if no real sensor available
        self.timer = self.create_timer(0.5, self.publish_mock_data)
        self.publisher = self.create_publisher(LaserScan, '/mock_scan', 10)

    def lidar_callback(self, msg):
        # Get the minimum distance (closest obstacle)
        if len(msg.ranges) > 0:
            valid_ranges = [r for r in msg.ranges if r != float('inf') and not math.isnan(r)]
            if valid_ranges:
                min_distance = min(valid_ranges)
                self.get_logger().info(f'Minimum distance: {min_distance:.2f}m')

                # Get distance at front (index closest to 0 degrees)
                front_idx = len(msg.ranges) // 2
                if front_idx < len(msg.ranges):
                    front_distance = msg.ranges[front_idx]
                    self.get_logger().info(f'Distance at front: {front_distance:.2f}m')

    def publish_mock_data(self):
        import random
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        # LIDAR parameters
        msg.angle_min = -math.pi / 2  # -90 degrees
        msg.angle_max = math.pi / 2    # 90 degrees
        msg.angle_increment = math.pi / 180  # 1 degree
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0

        # Generate mock ranges (simulating a room with obstacles)
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = []

        for i in range(num_readings):
            angle = msg.angle_min + i * msg.angle_increment
            # Simulate a wall at 2m with some obstacles
            distance = 2.0 + random.uniform(-0.2, 0.2)
            msg.ranges.append(distance)

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_reader = LidarReader()
    rclpy.spin(lidar_reader)
    lidar_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Camera Reader Node
Create `~/ros2_ws/src/sensor_examples/sensor_reader/camera_reader.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraReader(Node):
    def __init__(self):
        super().__init__('camera_reader')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()

        # Timer to simulate camera data if no real camera available
        self.timer = self.create_timer(0.1, self.publish_mock_image)
        self.publisher = self.create_publisher(Image, '/mock_camera/image_raw', 10)

    def camera_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Simple processing: detect edges using Canny
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Display the original and processed images
            cv2.imshow("Camera Feed", cv_image)
            cv2.imshow("Edges", edges)
            cv2.waitKey(1)

            self.get_logger().info(f'Received image: {cv_image.shape[1]}x{cv_image.shape[0]}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def publish_mock_image(self):
        # Create a mock image (colorful pattern)
        height, width = 480, 640
        img = np.zeros((height, width, 3), dtype=np.uint8)

        # Add some colorful shapes
        cv2.rectangle(img, (50, 50), (200, 200), (255, 0, 0), -1)  # Blue
        cv2.circle(img, (400, 150), 75, (0, 255, 0), -1)  # Green
        cv2.putText(img, 'Mock Camera', (50, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Convert to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = 'camera_frame'

        self.publisher.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    camera_reader = CameraReader()
    rclpy.spin(camera_reader)
    camera_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Actuator Control Examples

### 1. Servo Controller Node
Create `~/ros2_ws/src/sensor_examples/sensor_reader/servo_controller.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Publisher for servo position commands
        self.servo_publisher = self.create_publisher(Float64, '/servo_position', 10)

        # Timer to send servo commands
        self.timer = self.create_timer(0.5, self.send_servo_command)
        self.command_index = 0

        # Predefined positions for demonstration
        self.positions = [0.0, 0.5, -0.5, 0.0, 1.0, -1.0, 0.0]  # radians

    def send_servo_command(self):
        if self.command_index < len(self.positions):
            msg = Float64()
            msg.data = self.positions[self.command_index]

            self.servo_publisher.publish(msg)
            self.get_logger().info(f'Sending servo position: {msg.data:.2f} rad')

            self.command_index += 1
        else:
            self.get_logger().info('Completed servo position sequence')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()
    rclpy.spin(servo_controller)
    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. AI Decision Node
Create `~/ros2_ws/src/sensor_examples/sensor_reader/ai_decision_node.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import math

class AIDecisionNode(Node):
    def __init__(self):
        super().__init__('ai_decision_node')

        # Subscribe to sensor data
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        # Publisher for actuator commands
        self.servo_publisher = self.create_publisher(Float64, '/servo_command', 10)

        self.get_logger().info('AI Decision Node initialized')

    def lidar_callback(self, msg):
        # Simple AI decision: if obstacle is close in front, turn servo to avoid
        if len(msg.ranges) > 0:
            # Get distance at front (center of scan)
            front_idx = len(msg.ranges) // 2
            if front_idx < len(msg.ranges):
                front_distance = msg.ranges[front_idx]

                # Decision logic
                if front_distance < 0.5:  # Obstacle within 50cm
                    # Turn servo to look left
                    command = -0.5  # radians
                    self.get_logger().info(f'Obstacle detected! Distance: {front_distance:.2f}m. Turning left.')
                elif front_distance > 1.0:  # Clear path ahead
                    # Turn servo to look forward
                    command = 0.0  # radians
                    self.get_logger().info(f'Clear path! Distance: {front_distance:.2f}m. Looking forward.')
                else:
                    # Look straight ahead
                    command = 0.0  # radians
                    self.get_logger().info(f'Normal distance: {front_distance:.2f}m. Looking ahead.')

                # Publish actuator command
                cmd_msg = Float64()
                cmd_msg.data = command
                self.servo_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    ai_decision_node = AIDecisionNode()
    rclpy.spin(ai_decision_node)
    ai_decision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Update setup.py
Update `~/ros2_ws/src/sensor_examples/setup.py` to include entry points:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'sensor_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Examples for sensor reading and actuator control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_reader = sensor_reader.imu_reader:main',
            'lidar_reader = sensor_reader.lidar_reader:main',
            'camera_reader = sensor_reader.camera_reader:main',
            'servo_controller = sensor_reader.servo_controller:main',
            'ai_decision_node = sensor_reader.ai_decision_node:main',
        ],
    },
)
```

## Building and Running Examples

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select sensor_reader
source ~/ros2_ws/install/setup.bash
```

### 2. Run Individual Examples
```bash
# Terminal 1: Run IMU reader
ros2 run sensor_reader imu_reader

# Terminal 2: Run LIDAR reader
ros2 run sensor_reader lidar_reader

# Terminal 3: Run camera reader
ros2 run sensor_reader camera_reader
```

### 3. Run AI Decision Example
```bash
# Terminal 1: Run AI decision node (will subscribe to mock sensor data)
ros2 run sensor_reader ai_decision_node

# Terminal 2: Run mock sensor data publishers
ros2 run sensor_reader lidar_reader  # This will publish mock LIDAR data
```

## Running the Module's Hands-on Lab

The hands-on lab for this module involves:
1. Reading sensor data from simulation (IMU, LIDAR, or camera)
2. Processing the sensor data with simple AI decision logic
3. Using the decisions to control an actuator (servo or motor)

Use the examples above as a foundation and combine them to create your own sensor-actuator integration project.

## Testing Your Setup

Verify your sensor/actuator setup:
```bash
# Check available sensor topics
ros2 topic list | grep -E "(imu|scan|camera|sensor)"

# Echo a sensor topic to see data
ros2 topic echo /mock_imu/data sensor_msgs/msg/Imu

# Check available actuator topics
ros2 topic list | grep -E "(servo|motor|actuator)"
```

## Troubleshooting

- If OpenCV is not found: `pip3 install opencv-python`
- If cv_bridge import fails: `sudo apt install ros-humble-cv-bridge`
- If sensor messages are not found: `sudo apt install ros-humble-sensor-msgs`
- For camera display issues: Make sure X11 forwarding is enabled if using SSH