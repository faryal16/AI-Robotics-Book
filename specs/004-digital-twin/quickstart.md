# Quickstart: Module 4: The Digital Twin

## Prerequisites

- Completed Module 1, 2, and 3
- Ubuntu 22.04 LTS with ROS 2 Humble installed
- Python 3.10+
- Git for version control
- Basic understanding of reinforcement learning concepts
- Unity 2022.3 LTS (for Unity components)

## Setup Simulation Environments

### 1. Install Gazebo (Garden)
```bash
# Add Gazebo repository
sudo apt update && sudo apt install wget
wget https://packages.osrfoundation.org/gazebo.gpg -O /tmp/gazebo.gpg
sudo cp /tmp/gazebo.gpg /usr/share/keyrings/
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install gz-garden
```

### 2. Install Gazebo ROS2 Bridge
```bash
sudo apt install ros-humble-ros-gz
sudo apt install ros-humble-ros-gz-bridge
sudo apt install ros-humble-ros-gz-sim
sudo apt install ros-humble-ros-gz-plugins
```

### 3. Install Unity (Optional for Unity components)
- Download Unity Hub from Unity website
- Install Unity 2022.3 LTS through Unity Hub
- Install Unity Robotics Package through Package Manager

### 4. Install Reinforcement Learning Dependencies
```bash
pip3 install stable-baselines3[extra] torch torchvision torchaudio
pip3 install gymnasium[box2d]
pip3 install numpy matplotlib pandas
```

## Basic Gazebo Physics Simulation

### 1. Create a Gazebo Package
```bash
mkdir -p ~/ros2_ws/src/digital_twin_examples
cd ~/ros2_ws/src/digital_twin_examples
ros2 pkg create --build-type ament_python gazebo_tutorials --dependencies rclpy std_msgs geometry_msgs sensor_msgs
```

### 2. Create a Simple Gazebo World
Create `~/ros2_ws/src/digital_twin_examples/gazebo_tutorials/worlds/simple_physics.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_physics">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sky -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Create a simple box that will fall due to gravity -->
    <model name="falling_box">
      <pose>0 0 2 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Create a static box as an obstacle -->
    <model name="obstacle">
      <pose>0.5 0 0.1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### 3. Create a Robot Arm Model
Create `~/ros2_ws/src/digital_twin_examples/gazebo_tutorials/models/simple_arm/model.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_arm">
    <!-- Base of the arm -->
    <link name="base_link">
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.2</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.2</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <!-- First joint (shoulder) -->
    <joint name="shoulder_joint" type="revolute">
      <parent>base_link</parent>
      <child>upper_arm_link</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>100</effort>
          <velocity>1</velocity>
        </limit>
      </axis>
      <pose relative_to="base_link">0 0 0.2 0 0 0</pose>
    </joint>

    <!-- Upper arm -->
    <link name="upper_arm_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.05</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.05</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.5</length>
          </cylinder>
        </geometry>
        <pose>0 0 0.25 0 0 0</pose>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.5</length>
          </cylinder>
        </geometry>
        <pose>0 0 0.25 0 0 0</pose>
      </collision>
    </link>

    <!-- Second joint (elbow) -->
    <joint name="elbow_joint" type="revolute">
      <parent>upper_arm_link</parent>
      <child>lower_arm_link</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>100</effort>
          <velocity>1</velocity>
        </limit>
      </axis>
      <pose relative_to="upper_arm_link">0 0 0.5 0 0 0</pose>
    </joint>

    <!-- Lower arm -->
    <link name="lower_arm_link">
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.02</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.02</iyy>
          <iyz>0</iyz>
          <izz>0.005</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.4</length>
          </cylinder>
        </geometry>
        <pose>0 0 0.2 0 0 0</pose>
        <material>
          <ambient>0.9 0.9 0.9 1</ambient>
          <diffuse>0.9 0.9 0.9 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.4</length>
          </cylinder>
        </geometry>
        <pose>0 0 0.2 0 0 0</pose>
      </collision>
    </link>

    <!-- Physics plugin for control -->
    <plugin filename="libgz-sim-joint-position-controller-system.so" name="gz::sim::systems::JointPositionController">
      <joint_name>shoulder_joint</joint_name>
    </plugin>

    <plugin filename="libgz-sim-joint-position-controller-system.so" name="gz::sim::systems::JointPositionController">
      <joint_name>elbow_joint</joint_name>
    </plugin>
  </model>
</sdf>
```

### 4. Sensor Simulation Node
Create `~/ros2_ws/src/digital_twin_examples/gazebo_tutorials/sensor_simulator.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from cv_bridge import CvBridge
import numpy as np
import math

class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')

        # Publishers for simulated sensors
        self.lidar_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # Timer for sensor simulation
        self.timer = self.create_timer(0.1, self.publish_sensor_data)
        self.bridge = CvBridge()
        self.time_step = 0.0

    def publish_sensor_data(self):
        self.publish_lidar_data()
        self.publish_imu_data()
        self.publish_camera_data()
        self.time_step += 0.1

    def publish_lidar_data(self):
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
            # Simulate a wall at 2m with some variation
            distance = 2.0 + 0.2 * math.sin(self.time_step * 2 + angle * 3)
            msg.ranges.append(distance)

        self.lidar_pub.publish(msg)

    def publish_imu_data(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulate IMU data with some motion
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(self.time_step * 0.5) * 0.1
        msg.orientation.w = math.cos(self.time_step * 0.5) * 0.1

        # Angular velocity
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.5 * math.cos(self.time_step * 0.5) * 0.1

        # Linear acceleration (including gravity)
        msg.linear_acceleration.x = 0.1 * math.cos(self.time_step)
        msg.linear_acceleration.y = 0.05 * math.sin(self.time_step)
        msg.linear_acceleration.z = 9.8 + 0.1 * math.sin(self.time_step * 2)

        self.imu_pub.publish(msg)

    def publish_camera_data(self):
        # Create a mock image (changing pattern over time)
        height, width = 480, 640
        img = np.zeros((height, width, 3), dtype=np.uint8)

        # Add moving shapes
        center_x = int(320 + 100 * math.sin(self.time_step))
        center_y = int(240 + 50 * math.cos(self.time_step * 0.7))
        cv2.circle(img, (center_x, center_y), 50, (255, 0, 0), -1)  # Blue circle

        # Add text with timestamp
        cv2.putText(img, f'Time: {self.time_step:.1f}s', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Convert to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = 'camera_frame'

        self.camera_pub.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    sensor_simulator = SensorSimulator()
    rclpy.spin(sensor_simulator)
    sensor_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5. Install OpenCV for Camera Simulation
```bash
pip3 install opencv-python
```

## Reinforcement Learning Example: Robot Arm Control

### 1. Create RL Package
```bash
mkdir -p ~/ros2_ws/src/digital_twin_examples/rl_examples
cd ~/ros2_ws/src/digital_twin_examples/rl_examples
touch __init__.py
```

### 2. Create Custom Gym Environment for Robot Arm
Create `~/ros2_ws/src/digital_twin_examples/rl_examples/arm_env.py`:

```python
import gymnasium as gym
import numpy as np
from gymnasium import spaces
import math

class RobotArmEnv(gym.Env):
    """
    Custom Environment for controlling a 2-DOF robot arm
    """
    def __init__(self):
        super(RobotArmEnv, self).__init__()

        # Define action space: joint velocities for 2 joints
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )

        # Define observation space: joint angles, velocities, end-effector position
        self.observation_space = spaces.Box(
            low=np.array([-np.pi, -np.pi, -5.0, -5.0, -2.0, -2.0, -2.0], dtype=np.float32),
            high=np.array([np.pi, np.pi, 5.0, 5.0, 2.0, 2.0, 2.0], dtype=np.float32),
            dtype=np.float32
        )

        # Robot arm parameters
        self.arm_lengths = [0.5, 0.4]  # Length of upper and lower arm
        self.dt = 0.1  # Time step

        # Target position
        self.target_pos = np.array([0.6, 0.6])

        # Initialize state
        self.reset()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Initialize random joint angles
        self.joint_angles = np.array([
            self.np_random.uniform(-np.pi/2, np.pi/2),
            self.np_random.uniform(-np.pi/2, np.pi/2)
        ])

        self.joint_velocities = np.zeros(2)

        # Calculate initial end-effector position
        self.end_effector_pos = self.forward_kinematics(self.joint_angles)

        observation = self._get_observation()
        info = {}

        return observation, info

    def step(self, action):
        # Update joint velocities based on action
        self.joint_velocities += action * self.dt
        # Limit velocities
        self.joint_velocities = np.clip(self.joint_velocities, -2.0, 2.0)

        # Update joint angles
        self.joint_angles += self.joint_velocities * self.dt

        # Keep joint angles within limits
        self.joint_angles = np.clip(self.joint_angles, -np.pi, np.pi)

        # Calculate new end-effector position
        self.end_effector_pos = self.forward_kinematics(self.joint_angles)

        # Calculate reward
        reward = self._calculate_reward()

        # Check if episode is done
        terminated = self._check_termination()
        truncated = False  # No time limit

        observation = self._get_observation()
        info = {}

        return observation, reward, terminated, truncated, info

    def _get_observation(self):
        """
        Get the current observation: [joint_angles, joint_velocities, end_effector_pos]
        """
        return np.concatenate([
            self.joint_angles,
            self.joint_velocities,
            self.end_effector_pos
        ])

    def forward_kinematics(self, angles):
        """
        Calculate end-effector position from joint angles
        """
        theta1, theta2 = angles

        # Position of elbow
        elbow_x = self.arm_lengths[0] * np.cos(theta1)
        elbow_y = self.arm_lengths[0] * np.sin(theta1)

        # Position of end-effector
        end_x = elbow_x + self.arm_lengths[1] * np.cos(theta1 + theta2)
        end_y = elbow_y + self.arm_lengths[1] * np.sin(theta1 + theta2)

        return np.array([end_x, end_y, 0.0])  # z=0 for 2D arm

    def _calculate_reward(self):
        """
        Calculate reward based on distance to target
        """
        distance_to_target = np.linalg.norm(self.end_effector_pos[:2] - self.target_pos)

        # Reward is negative distance (so closer is better)
        reward = -distance_to_target

        # Bonus for getting very close to target
        if distance_to_target < 0.1:
            reward += 10

        # Penalty for large joint velocities (smooth movement)
        velocity_penalty = -0.01 * np.sum(np.abs(self.joint_velocities))
        reward += velocity_penalty

        return reward

    def _check_termination(self):
        """
        Check if the episode should terminate
        """
        distance_to_target = np.linalg.norm(self.end_effector_pos[:2] - self.target_pos)
        return distance_to_target < 0.05  # Success if within 5cm of target

    def render(self, mode='human'):
        """
        Render the environment (for visualization)
        """
        import matplotlib.pyplot as plt

        # Calculate arm positions
        theta1, theta2 = self.joint_angles
        elbow_x = self.arm_lengths[0] * np.cos(theta1)
        elbow_y = self.arm_lengths[0] * np.sin(theta1)
        end_x = elbow_x + self.arm_lengths[1] * np.cos(theta1 + theta2)
        end_y = elbow_y + self.arm_lengths[1] * np.sin(theta1 + theta2)

        plt.figure(figsize=(8, 8))
        plt.plot([0, elbow_x, end_x], [0, elbow_y, end_y], 'b-', linewidth=5, label='Arm')
        plt.plot(end_x, end_y, 'ro', markersize=10, label='End Effector')
        plt.plot(self.target_pos[0], self.target_pos[1], 'g*', markersize=15, label='Target')
        plt.xlim(-1.5, 1.5)
        plt.ylim(-0.5, 1.5)
        plt.grid(True)
        plt.legend()
        plt.title('Robot Arm Environment')
        plt.show()
```

### 3. Create RL Training Script
Create `~/ros2_ws/src/digital_twin_examples/rl_examples/train_arm.py`:

```python
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback
import os

from arm_env import RobotArmEnv

def train_robot_arm():
    # Create the environment
    env = RobotArmEnv()

    # Create a vectorized environment (for stable baselines)
    vec_env = make_vec_env(lambda: RobotArmEnv(), n_envs=1)

    # Create the RL agent (PPO - Proximal Policy Optimization)
    model = PPO(
        "MlpPolicy",
        vec_env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        tensorboard_log="./arm_tensorboard/"
    )

    # Train the agent
    print("Starting training...")
    model.learn(total_timesteps=50000)

    # Save the trained model
    model.save("robot_arm_ppo_model")
    print("Model saved!")

    # Test the trained agent
    obs, _ = env.reset()
    for i in range(1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, rewards, terminated, truncated, info = env.step(action)

        if terminated or truncated:
            print(f"Episode finished after {i+1} steps")
            obs, _ = env.reset()

    print("Training completed!")

if __name__ == "__main__":
    train_robot_arm()
```

### 4. Update setup.py
Update `~/ros2_ws/src/digital_twin_examples/setup.py` to include entry points:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'gazebo_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include worlds
        (os.path.join('share', package_name, 'worlds'), glob('gazebo_tutorials/worlds/*.world')),
        # Include models
        (os.path.join('share', package_name, 'models'), glob('gazebo_tutorials/models/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Examples for digital twin simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_simulator = gazebo_tutorials.sensor_simulator:main',
        ],
    },
)
```

## Running Examples

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select gazebo_tutorials
source ~/ros2_ws/install/setup.bash
```

### 2. Run Gazebo with Simple World
```bash
# Run Gazebo with the simple physics world
gz sim -r ~/ros2_ws/src/digital_twin_examples/gazebo_tutorials/worlds/simple_physics.world
```

### 3. Run Sensor Simulation
```bash
# In a new terminal, run the sensor simulator
ros2 run gazebo_tutorials sensor_simulator
```

### 4. Train the RL Agent
```bash
# Install gymnasium if not already installed
pip3 install gymnasium

# Run the RL training
cd ~/ros2_ws/src/digital_twin_examples/rl_examples
python3 train_arm.py
```

## Running the Module's Hands-on Lab

The hands-on lab for this module involves:
1. Setting up physics simulation in Gazebo with accurate gravity and collisions
2. Creating sensor simulations (LiDAR, Depth Camera, IMU)
3. Implementing a reinforcement learning loop to control a simulated robot arm
4. Training the agent to perform specific tasks

For the lab, you'll combine the concepts learned in the examples above to create a complete digital twin environment with RL-based control.

## Testing Your Setup

Verify your digital twin setup:
```bash
# Check if Gazebo is properly installed
gz --version

# Check if Gazebo ROS2 bridge is available
ros2 pkg list | grep gz

# Check if RL dependencies are installed
python3 -c "import stable_baselines3; print('SB3 installed')"
python3 -c "import gymnasium; print('Gymnasium installed')"
```

## Troubleshooting

- If Gazebo doesn't launch: Check graphics drivers and X11 forwarding if using SSH
- If ROS-Gazebo bridge doesn't work: Verify installation of ros-humble-ros-gz packages
- If RL training fails: Check Python dependencies and environment setup
- For Unity components: Ensure Unity Robotics Package is properly installed