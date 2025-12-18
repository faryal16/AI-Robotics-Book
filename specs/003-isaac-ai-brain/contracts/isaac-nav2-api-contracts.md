# API Contracts: Module 3: The AI-Robot Brain (NVIDIA Isaac™)

## Overview
This document defines the conceptual API contracts and interfaces for the Isaac Sim and Nav2 learning module. These represent the interfaces for simulation, perception, and navigation that students will learn to use and implement.

## Isaac Sim Interface Specifications

### Isaac Sim Scene Interface
```python
from pxr import Usd, UsdGeom, Gf
import carb

class IsaacSimSceneInterface:
    """
    Interface for Isaac Sim scene management
    """
    def create_scene(self, scene_name: str) -> object:
        """Create a new scene in Isaac Sim"""
        pass

    def add_robot(self, scene, robot_config: dict) -> object:
        """Add a robot to the scene"""
        pass

    def add_object(self, scene, object_config: dict) -> object:
        """Add an object to the scene"""
        pass

    def configure_sensors(self, robot, sensor_configs: list) -> list:
        """Configure sensors for a robot"""
        pass

    def run_simulation(self, scene, duration: float):
        """Run simulation for specified duration"""
        pass

    def get_simulation_data(self, scene) -> dict:
        """Get simulation data including robot states and sensor readings"""
        pass
```

### Isaac Sim Sensor Interface
```python
class IsaacSimSensorInterface:
    """
    Interface for Isaac Sim sensor simulation
    """
    def create_rgb_camera(self, config: dict) -> object:
        """Create RGB camera sensor"""
        pass

    def create_depth_camera(self, config: dict) -> object:
        """Create depth camera sensor"""
        pass

    def create_lidar(self, config: dict) -> object:
        """Create LIDAR sensor"""
        pass

    def get_sensor_data(self, sensor) -> object:
        """Get data from sensor"""
        pass

    def configure_sensor_parameters(self, sensor, params: dict):
        """Configure sensor-specific parameters"""
        pass
```

## Isaac ROS Interface Specifications

### VSLAM Interface
```python
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class IsaacROSVisualSLAMInterface:
    """
    Interface for Isaac ROS Visual SLAM functionality
    """
    def process_stereo_images(self, left_image: Image, right_image: Image,
                              left_info: CameraInfo, right_info: CameraInfo) -> PoseStamped:
        """Process stereo images for visual odometry"""
        pass

    def process_rgbd_data(self, rgb_image: Image, depth_image: Image,
                          camera_info: CameraInfo) -> PoseStamped:
        """Process RGB-D data for visual odometry"""
        pass

    def build_map(self, poses: list, images: list) -> object:
        """Build map from poses and images"""
        pass

    def localize_robot(self, current_image: Image, map_data: object) -> PoseStamped:
        """Localize robot in existing map"""
        pass

    def get_tracking_status(self) -> str:
        """Get current tracking status (OK, LOST, etc.)"""
        pass
```

### Isaac ROS Perception Interface
```python
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose

class IsaacROSPeceptionInterface:
    """
    Interface for Isaac ROS perception functionality
    """
    def detect_objects(self, image: Image) -> Detection2DArray:
        """Detect objects in image"""
        pass

    def estimate_depth(self, image: Image) -> object:  # Depth image or point cloud
        """Estimate depth from single or stereo images"""
        pass

    def segment_scene(self, image: Image) -> object:  # Segmentation mask
        """Perform semantic segmentation on image"""
        pass

    def track_objects(self, image: Image, objects: list) -> list:
        """Track objects across frames"""
        pass
```

## Nav2 Navigation Interface Specifications

### Path Planning Interface
```python
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

class Nav2PathPlanningInterface:
    """
    Interface for Nav2 path planning
    """
    def compute_path_to_pose(self, start: PoseStamped, goal: PoseStamped) -> Path:
        """Compute path from start to goal pose"""
        pass

    def compute_path_through_poses(self, poses: list) -> Path:
        """Compute path through multiple waypoints"""
        pass

    def get_global_costmap(self) -> object:
        """Get the global costmap"""
        pass

    def get_local_costmap(self) -> object:
        """Get the local costmap"""
        pass

    def set_planner_type(self, planner_name: str):
        """Set the path planner type"""
        pass
```

### Navigation Execution Interface
```python
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class Nav2NavigationInterface:
    """
    Interface for Nav2 navigation execution
    """
    def navigate_to_pose(self, goal: PoseStamped) -> bool:
        """Navigate to a specific pose"""
        pass

    def navigate_through_poses(self, goals: list) -> bool:
        """Navigate through a series of poses"""
        pass

    def follow_path(self, path: Path) -> bool:
        """Follow a predefined path"""
        pass

    def cancel_navigation(self):
        """Cancel current navigation task"""
        pass

    def get_navigation_status(self) -> String:
        """Get current navigation status"""
        pass

    def get_current_pose(self) -> PoseStamped:
        """Get current robot pose"""
        pass
```

### Humanoid-Specific Navigation Interface
```python
from geometry_msgs.msg import PoseStamped, Twist

class HumanoidNavigationInterface:
    """
    Interface for humanoid-specific navigation
    """
    def compute_bipedal_path(self, start: PoseStamped, goal: PoseStamped) -> Path:
        """Compute path considering bipedal movement constraints"""
        pass

    def generate_footstep_plan(self, path: Path) -> list:
        """Generate footstep plan for bipedal locomotion"""
        pass

    def execute_bipedal_motion(self, footsteps: list) -> bool:
        """Execute bipedal motion based on footsteps"""
        pass

    def maintain_balance(self, twist_cmd: Twist) -> Twist:
        """Adjust twist command to maintain balance"""
        pass

    def step_adjustment_needed(self, current_pose: PoseStamped, planned_pose: PoseStamped) -> bool:
        """Determine if step adjustment is needed for balance"""
        pass
```

## Synthetic Data Generation Interface

### Synthetic Dataset Interface
```python
class SyntheticDataInterface:
    """
    Interface for synthetic data generation using Isaac Sim
    """
    def generate_training_dataset(self, scene_config: dict, num_samples: int,
                                 data_type: str) -> str:
        """Generate synthetic training dataset"""
        pass

    def configure_data_augmentation(self, params: dict):
        """Configure data augmentation parameters"""
        pass

    def label_synthetic_data(self, raw_data: object) -> object:
        """Generate accurate labels for synthetic data"""
        pass

    def validate_synthetic_data(self, dataset_path: str) -> dict:
        """Validate synthetic dataset quality"""
        pass

    def export_dataset(self, dataset_path: str, format_type: str) -> str:
        """Export dataset in specified format"""
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
  "technologyFocus": ["enum (ISAAC_SIM|ISAAC_ROS|NAV2|VSLAM|PERCEPTION)"],
  "simulationRequired": "boolean",
  "gpuRequired": "boolean",
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (DRAFT|REVIEW|PUBLISHED)"
}
```

### Isaac Sim Scene Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "name": "string",
  "description": "string",
  "environmentType": "enum (INDOOR|OUTDOOR|LAB|WAREHOUSE|OTHER)",
  "objects": ["object"],
  "sensors": ["object"],
  "physicsSettings": "object",
  "renderingSettings": "object",
  "simulationDuration": "float (seconds)",
  "learningObjectives": ["string"],
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (ACTIVE|ARCHIVED)"
}
```

### Perception Model Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "name": "string",
  "type": "enum (CLASSIFICATION|DETECTION|SEGMENTATION|VSLAM|OTHER)",
  "architecture": "string",
  "inputType": "string",
  "outputType": "string",
  "trainingData": "string",
  "syntheticDataRatio": "float (0.0-1.0)",
  "accuracyMetrics": "object",
  "exampleCode": "string (Python code)",
  "trainingInstructions": "string (Markdown format)",
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (ACTIVE|ARCHIVED)"
}
```

### Navigation Configuration Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "name": "string",
  "robotType": "enum (WHEELED|BIPEDAL|QUADRUPED|OTHER)",
  "plannerType": "enum (GLOBAL_PLANNER|LOCAL_PLANNER|CONTROLLER)",
  "parameters": "object",
  "mapType": "enum (TOPOLOGICAL|GRID|MESH|OTHER)",
  "obstacleHandling": "string",
  "exampleConfig": "string (YAML format)",
  "tuningGuidance": "string (Markdown format)",
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (ACTIVE|ARCHIVED)"
}
```

## Error Handling Patterns

### Isaac Sim Error Interface
```python
class IsaacSimError(Exception):
    """Base exception for Isaac Sim operations"""
    def __init__(self, operation: str, message: str, error_code: str = None):
        self.operation = operation
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)

# Example usage:
try:
    # Isaac Sim operation
    pass
except IsaacSimError as e:
    node.get_logger().error(f'Isaac Sim Error ({e.operation}): {e.message} (Code: {e.error_code})')
```

### Navigation Error Interface
```python
class NavigationError(Exception):
    """Base exception for navigation operations"""
    def __init__(self, navigation_phase: str, message: str, error_code: str = None):
        self.navigation_phase = navigation_phase  # planning, execution, localization, etc.
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)

# Example usage:
try:
    # Navigation operation
    pass
except NavigationError as e:
    node.get_logger().error(f'Navigation Error ({e.navigation_phase}): {e.message} (Code: {e.error_code})')
```

## Validation Interfaces

### Isaac Sim Scene Validation Interface
```python
class IsaacSimSceneValidatorInterface:
    """
    Interface for validating Isaac Sim scenes
    """
    def validate_scene_composition(self, scene_config: dict) -> dict:
        """Validate scene composition and return validation report"""
        pass

    def check_sensor_placement(self, robot_config: dict) -> dict:
        """Check if sensors are properly placed on robot"""
        pass

    def validate_physics_properties(self, scene_config: dict) -> dict:
        """Validate physics properties of objects in scene"""
        pass

    def check_rendering_quality(self, scene_config: dict) -> dict:
        """Check if rendering settings are appropriate"""
        pass
```

### Navigation Configuration Validation Interface
```python
class NavigationConfigValidatorInterface:
    """
    Interface for validating navigation configurations
    """
    def validate_planner_parameters(self, config: dict) -> dict:
        """Validate planner-specific parameters"""
        pass

    def check_costmap_settings(self, config: dict) -> dict:
        """Validate costmap configuration"""
        pass

    def validate_controller_settings(self, config: dict) -> dict:
        """Validate controller configuration"""
        pass

    def check_humanoid_specific_params(self, config: dict) -> dict:
        """Validate humanoid-specific navigation parameters"""
        pass
```

### Synthetic Data Quality Interface
```python
class SyntheticDataQualityInterface:
    """
    Interface for validating synthetic data quality
    """
    def check_data_diversity(self, dataset_path: str) -> dict:
        """Check diversity of synthetic dataset"""
        pass

    def validate_label_accuracy(self, dataset_path: str) -> dict:
        """Validate accuracy of synthetic labels"""
        pass

    def assess_realism(self, synthetic_data: object, real_data: object) -> dict:
        """Assess realism of synthetic data compared to real data"""
        pass

    def validate_dataset_completeness(self, dataset_path: str) -> dict:
        """Validate that dataset is complete and not missing samples"""
        pass
```