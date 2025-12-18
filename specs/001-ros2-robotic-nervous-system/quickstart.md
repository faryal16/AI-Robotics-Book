# Quickstart: Module 1: The Robotic Nervous System (ROS 2)

## Prerequisites

- Ubuntu 22.04 LTS (recommended for ROS 2 Humble)
- ROS 2 Humble Hawksbill installed
- Python 3.10+
- Git for version control
- Basic Python programming knowledge

## Setup ROS 2 Environment

### 1. Install ROS 2 Humble
```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo apt install python3-colcon-common-extensions
```

### 2. Source ROS 2 Environment
```bash
source /opt/ros/humble/setup.bash
```

To make this permanent, add to your `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Creating Your First ROS 2 Package

### 1. Create a Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Create a Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_tutorials --dependencies rclpy std_msgs geometry_msgs
```

## Basic ROS 2 Examples

### 1. Simple Publisher Node
Create `~/ros2_ws/src/my_robot_tutorials/my_robot_tutorials/simple_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Simple Subscriber Node
Create `~/ros2_ws/src/my_robot_tutorials/my_robot_tutorials/simple_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Make Python Files Executable and Update setup.py
```bash
chmod +x ~/ros2_ws/src/my_robot_tutorials/my_robot_tutorials/simple_publisher.py
chmod +x ~/ros2_ws/src/my_robot_tutorials/my_robot_tutorials/simple_subscriber.py
```

Update `~/ros2_ws/src/my_robot_tutorials/setup.py` to include the entry points:

```python
entry_points={
    'console_scripts': [
        'simple_publisher = my_robot_tutorials.simple_publisher:main',
        'simple_subscriber = my_robot_tutorials.simple_subscriber:main',
    ],
},
```

## Building and Running Examples

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_tutorials
```

### 2. Source the Workspace
```bash
source ~/ros2_ws/install/setup.bash
```

### 3. Run the Publisher and Subscriber
Terminal 1:
```bash
ros2 run my_robot_tutorials simple_publisher
```

Terminal 2:
```bash
ros2 run my_robot_tutorials simple_subscriber
```

## Creating a URDF Model

### 1. Create URDF Directory
```bash
mkdir -p ~/ros2_ws/src/my_robot_tutorials/urdf
```

### 2. Create Simple Humanoid URDF
Create `~/ros2_ws/src/my_robot_tutorials/urdf/simple_humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="head_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_arm">
    <visual>
      <geometry>
        <box size="0.05 0.1 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_arm"/>
    <origin xyz="0.15 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

### 3. Visualize the URDF
```bash
# Install visualization tools if not already installed
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui

# Visualize the URDF
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/rviz2/default.rviz
```

In RViz2:
1. Add a RobotModel display
2. Set the Robot Description to "robot_description"
3. In another terminal, run:
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui ~/ros2_ws/src/my_robot_tutorials/urdf/simple_humanoid.urdf
```

## Running the Module's Hands-on Lab

The hands-on lab for this module involves creating a humanoid URDF model and writing a ROS 2 node to control a joint. Follow the URDF example above as a starting point and extend it to create your own humanoid model with multiple joints, then write a ROS 2 node to control one of the joints.

## Testing Your Setup

Verify your ROS 2 installation:
```bash
# Check ROS 2 version
ros2 --version

# List available topics
ros2 topic list

# List available services
ros2 service list
```

## Troubleshooting

- If you get "command not found" errors, make sure you've sourced the ROS 2 environment: `source /opt/ros/humble/setup.bash`
- If packages don't build, ensure all dependencies are installed: `rosdep install --from-paths src --ignore-src -r -y`
- For Python package issues, make sure you're using the correct Python environment