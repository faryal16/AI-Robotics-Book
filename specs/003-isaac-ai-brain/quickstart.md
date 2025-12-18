# Quickstart: Module 3: The AI-Robot Brain (NVIDIA Isaac™)

## Prerequisites

- Completed Module 1 and 2
- Ubuntu 22.04 LTS with ROS 2 Humble installed
- NVIDIA GPU with CUDA support (recommended: RTX 3080 or better)
- NVIDIA drivers installed
- Python 3.10+
- Git for version control

## Setup Isaac Ecosystem

### 1. Install NVIDIA Drivers
```bash
# Check if NVIDIA GPU is detected
lspci | grep -i nvidia

# Install NVIDIA drivers
sudo apt update
sudo apt install nvidia-driver-535  # Or latest compatible driver
sudo reboot
```

### 2. Verify CUDA Installation
```bash
# Check CUDA version
nvidia-smi

# Install CUDA toolkit if not already installed
sudo apt install nvidia-cuda-toolkit
```

### 3. Install Isaac Sim (Isaac Sim Omniverse)
Follow the official NVIDIA Isaac Sim installation guide:
- Download Isaac Sim from NVIDIA Developer website
- Extract and run the installation
- Ensure Isaac Sim can connect to Omniverse

### 4. Install Isaac ROS Dependencies
```bash
# Install Isaac ROS packages
sudo apt update
sudo apt install ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-gems
sudo apt install ros-humble-isaac-ros-visual-odometry
sudo apt install ros-humble-isaac-ros-point-cloud-transport
```

### 5. Install Navigation2 (Nav2)
```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-gui-plugins
```

## Basic Isaac Sim Concepts

### 1. Launch Isaac Sim
```bash
# Navigate to Isaac Sim directory and launch
cd /path/to/isaac-sim
./isaac-sim.launch.sh
```

### 2. Basic Isaac Sim Python API
Create a simple script to interact with Isaac Sim:

```python
# simple_isaac_sim.py
import omni
from pxr import Gf, UsdGeom, Sdf

def create_simple_scene():
    """Create a simple scene with a robot and obstacles"""

    # Get the current stage
    stage = omni.usd.get_context().get_stage()

    # Create a robot (simplified representation)
    robot_path = Sdf.Path("/World/Robot")
    robot_xform = UsdGeom.Xform.Define(stage, robot_path)

    # Create a simple box as the robot body
    robot_body_path = robot_path.AppendChild("Body")
    robot_body = UsdGeom.Cube.Define(stage, robot_body_path)
    robot_body.GetSizeAttr().Set(0.5)

    # Create an obstacle
    obstacle_path = Sdf.Path("/World/Obstacle")
    obstacle_xform = UsdGeom.Xform.Define(stage, obstacle_path)
    obstacle_xform.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 0.0))

    obstacle_body = UsdGeom.Cube.Define(stage, obstacle_path.AppendChild("Body"))
    obstacle_body.GetSizeAttr().Set(1.0)

    print("Simple scene created with robot and obstacle")

# To run this in Isaac Sim, use the Isaac Sim Python console
```

## Isaac ROS VSLAM Example

### 1. Create a VSLAM Package
```bash
mkdir -p ~/ros2_ws/src/vslam_examples
cd ~/ros2_ws/src/vslam_examples
ros2 pkg create --build-type ament_python vslam_tutorial --dependencies rclpy sensor_msgs geometry_msgs cv_bridge image_transport message_filters
```

### 2. Visual Odometry Node
Create `~/ros2_ws/src/vslam_examples/vslam_tutorial/visual_odometry.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

class VisualOdometryNode(Node):
    def __init__(self):
        super().__init__('visual_odometry_node')

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Store previous frame and pose
        self.prev_frame = None
        self.current_pose = np.eye(4)

        # Camera parameters (these would normally come from camera_info topic)
        self.camera_matrix = np.array([
            [554.25, 0.0, 320.5],
            [0.0, 554.25, 240.5],
            [0.0, 0.0, 1.0]
        ])

        # Create subscribers for stereo images or monocular with IMU
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for estimated pose
        self.pose_pub = self.create_publisher(PoseStamped, '/visual_odom/pose', 10)

        self.get_logger().info('Visual Odometry Node initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            current_frame = self.bridge.imgmsg_to_cv2(msg, "mono8")

            if self.prev_frame is not None:
                # Compute visual odometry using feature matching
                pose_increment = self.compute_vo(self.prev_frame, current_frame)

                # Update current pose
                self.current_pose = self.current_pose @ pose_increment

                # Publish pose
                self.publish_pose()

            self.prev_frame = current_frame

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def compute_vo(self, prev_frame, curr_frame):
        # Simple feature-based visual odometry
        # Detect features in both frames
        detector = cv2.ORB_create(nfeatures=1000)
        prev_kp, prev_desc = detector.detectAndCompute(prev_frame, None)
        curr_kp, curr_desc = detector.detectAndCompute(curr_frame, None)

        if prev_desc is None or curr_desc is None:
            return np.eye(4)

        # Match features
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = matcher.match(prev_desc, curr_desc)

        # Sort matches by distance
        matches = sorted(matches, key=lambda x: x.distance)

        # Keep only good matches
        good_matches = matches[:50]  # Keep top 50 matches

        if len(good_matches) < 10:
            return np.eye(4)  # Not enough matches

        # Extract corresponding points
        prev_pts = np.float32([prev_kp[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        curr_pts = np.float32([curr_kp[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        # Compute essential matrix and pose
        E, mask = cv2.findEssentialMat(
            curr_pts, prev_pts, self.camera_matrix,
            method=cv2.RANSAC, prob=0.999, threshold=1.0
        )

        if E is not None:
            # Decompose essential matrix
            _, R, t, _ = cv2.recoverPose(E, curr_pts, prev_pts, self.camera_matrix)

            # Create transformation matrix
            pose_increment = np.eye(4)
            pose_increment[:3, :3] = R
            pose_increment[:3, 3] = t.flatten()

            return pose_increment
        else:
            return np.eye(4)

    def publish_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Extract position and orientation from transformation matrix
        position = self.current_pose[:3, 3]
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])

        # Convert rotation matrix to quaternion
        R = self.current_pose[:3, :3]
        qw, qx, qy, qz = self.rotation_matrix_to_quaternion(R)
        pose_msg.pose.orientation.w = qw
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz

        self.pose_pub.publish(pose_msg)

    def rotation_matrix_to_quaternion(self, R):
        # Convert 3x3 rotation matrix to quaternion
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                qw = (R[2, 1] - R[1, 2]) / s
                qx = 0.25 * s
                qy = (R[0, 1] + R[1, 0]) / s
                qz = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                qw = (R[0, 2] - R[2, 0]) / s
                qx = (R[0, 1] + R[1, 0]) / s
                qy = 0.25 * s
                qz = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                qw = (R[1, 0] - R[0, 1]) / s
                qx = (R[0, 2] + R[2, 0]) / s
                qy = (R[1, 2] + R[2, 1]) / s
                qz = 0.25 * s

        return qw, qx, qy, qz

def main(args=None):
    rclpy.init(args=args)
    visual_odom_node = VisualOdometryNode()
    rclpy.spin(visual_odom_node)
    visual_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Nav2 Humanoid Path Planning Example

### 1. Create Nav2 Configuration for Humanoid
Create `~/ros2_ws/src/vslam_examples/vslam_tutorial/launch/humanoid_nav2.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    map_yaml_file = LaunchConfiguration('map_yaml_file')

    # Parameters
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    # Launch configuration
    bringup_launch_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch']
    )

    # Create the launch description
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'))

    ld.add_action(
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start the Nav2 stack'))

    ld.add_action(
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('vslam_tutorial'), 'config', 'humanoid_nav2_params.yaml']),
            description='Full path to the Nav2 parameters file'))

    ld.add_action(
        DeclareLaunchArgument(
            'bt_xml_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml']),
            description='Full path to the behavior tree xml file to use'))

    ld.add_action(
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('vslam_tutorial'), 'maps', 'simple_map.yaml']),
            description='Full path to the map file to load'))

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml_file},
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}],
        output='screen'
    )

    # Add nodes to launch description
    ld.add_action(map_server)
    ld.add_action(lifecycle_manager)

    return ld
```

### 2. Humanoid-Specific Nav2 Parameters
Create `~/ros2_ws/src/vslam_examples/vslam_tutorial/config/humanoid_nav2_params.yaml`:

```yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.1
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

amcl_map_client:
  ros__parameters:
    use_sim_time: False

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: False

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Bipedal-specific navigation behavior tree
    # Humanoid has different capabilities than wheeled robots
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_truncate_path_local_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_condition_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: False

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    # Humanoid-specific controller for bipedal movement
    # Different from differential drive controllers
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid Follow Path controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_horizon: 1.0
      frequency: 20.0
      velocity_samples: 1
      model_dt: 0.05
      batch_size: 2000
      vx_std: 0.2
      vy_std: 0.05
      wz_std: 0.3
      vx_max: 0.3
      vx_min: -0.1
      vy_max: 0.1
      wz_max: 0.5
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
      critics: ["BaseObstacleCritic", "GoalCritic", "PathAlignCritic", "PathFollowCritic", "PathProgressCritic", "PreferForwardCritic"]
      BaseObstacleCritic:
        plugin: "nav2_mppi_controller::BaseObstacleCritic"
        threshold_to_reject: 0.1
        data_type: "costmap"
      GoalCritic:
        plugin: "nav2_mppi_controller::GoalCritic"
        threshold: 0.5
        angle_threshold: 0.5
        linear_scaling_factor: 1.0
        angular_scaling_factor: 1.0
      PathAlignCritic:
        plugin: "nav2_mppi_controller::PathAlignCritic"
        cost_power: 1
        cost_scale: 2.0
        threshold_to_angle: 1.0
        max_path_occupancy_ratio: 0.5
      PathFollowCritic:
        plugin: "nav2_mppi_controller::PathFollowCritic"
        cost_power: 1
        cost_scale: 2.0
        threshold_to_angle: 1.0
      PathProgressCritic:
        plugin: "nav2_mppi_controller::PathProgressCritic"
        cost_power: 1
        threshold_to_update: 0.5
      PreferForwardCritic:
        plugin: "nav2_mppi_controller::PreferForwardCritic"
        cost_power: 1
        linear_forward_penalty: 0.05
        linear_sideways_penalty: 4.0
        angular_penalty: 1.0

controller_server_rclcpp_node:
  ros__parameters:
    use_sim_time: False

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: False
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      # Humanoid-specific costmap for bipedal navigation
      # Consider foot placement and balance
      robot_radius: 0.3  # Humanoid radius
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 8
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
  local_costmap_client:
    ros__parameters:
      use_sim_time: False
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: False

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: False
      robot_radius: 0.3  # Humanoid radius
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
  global_costmap_client:
    ros__parameters:
      use_sim_time: False
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: False

map_server:
  ros__parameters:
    use_sim_time: False
    yaml_filename: ""

map_saver:
  ros__parameters:
    use_sim_time: False
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    # Humanoid-specific path planner
    # Consider bipedal movement constraints
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: False

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    recovery_plugin_types: ["nav2_recoveries::Spin", "nav2_recoveries::BackUp", "nav2_recoveries::Wait"]
    spin:
      plugin: "nav2_recoveries::Spin"
      ideal_linear_velocity: 0.0
      ideal_angular_velocity: 1.0
      commanded_linear_velocity: 0.0
      commanded_angular_velocity: 1.0
      tolerance: 1.571
      sampling_frequency: 20.0
    backup:
      plugin: "nav2_recoveries::BackUp"
      linear_distance: -0.15
      sim_frequency: 50.0
      cmd_topic: cmd_vel
    wait:
      plugin: "nav2_recoveries::Wait"
      wait_duration: 1.0

recoveries_server_rclcpp_node:
  ros__parameters:
    use_sim_time: False

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      wait_time: 1
```

### 3. Update setup.py
Update `~/ros2_ws/src/vslam_examples/setup.py` to include entry points:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'vslam_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include map files
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Examples for Isaac Sim and Nav2 integration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_odometry = vslam_tutorial.visual_odometry:main',
        ],
    },
)
```

## Building and Running Examples

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select vslam_tutorial
source ~/ros2_ws/install/setup.bash
```

### 2. Run Visual Odometry Example
```bash
# This example requires camera data
# You can simulate camera data or use a real camera
ros2 run vslam_tutorial visual_odometry
```

### 3. Launch Nav2 for Humanoid
```bash
# Create a simple map for testing
mkdir -p ~/ros2_ws/src/vslam_examples/vslam_tutorial/maps

# Create a simple map YAML file
cat > ~/ros2_ws/src/vslam_examples/vslam_tutorial/maps/simple_map.yaml << EOF
image: simple_map.pgm
mode: trinary
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
EOF

# Launch the Nav2 stack for humanoid
ros2 launch vslam_tutorial humanoid_nav2.launch.py
```

## Running the Module's Hands-on Lab

The hands-on lab for this module involves:
1. Using Isaac Sim to create a humanoid robot simulation
2. Generating synthetic data for perception model training
3. Implementing VSLAM using Isaac ROS
4. Setting up Nav2 for bipedal humanoid navigation

For the lab, you'll combine the concepts learned in the examples above to create a complete perception and navigation pipeline for a humanoid robot in simulation.

## Testing Your Setup

Verify your Isaac/Nav2 setup:
```bash
# Check if Isaac ROS packages are available
ros2 pkg list | grep isaac

# Check if Nav2 packages are available
ros2 pkg list | grep nav2

# Check available topics for navigation
ros2 topic list | grep -E "(nav|move|cmd)"
```

## Troubleshooting

- If Isaac Sim doesn't launch: Verify NVIDIA GPU and driver installation
- If CUDA errors occur: Check CUDA version compatibility
- If Nav2 fails to start: Verify parameter file paths and ROS 2 installation
- For permission issues: Ensure proper ROS 2 environment sourcing