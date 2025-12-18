# API Contracts: Module 4: The Digital Twin

## Overview
This document defines the conceptual API contracts and interfaces for the Digital Twin learning module. These represent the interfaces for physics simulation, rendering, sensor simulation, and reinforcement learning that students will learn to use and implement.

## Gazebo Simulation Interface Specifications

### Gazebo Environment Interface
```python
from gz.msgs11 import WorldStatistics, Contact
from gz.sim7 import Simulator, EntityComponentManager

class GazeboEnvironmentInterface:
    """
    Interface for Gazebo simulation environment management
    """
    def create_world(self, world_config: dict) -> object:
        """Create a new simulation world"""
        pass

    def add_model(self, world, model_config: dict) -> object:
        """Add a model to the simulation"""
        pass

    def configure_physics(self, world, physics_params: dict):
        """Configure physics engine parameters"""
        pass

    def run_simulation_step(self, world, step_size: float):
        """Run a single simulation step"""
        pass

    def get_simulation_stats(self, world) -> WorldStatistics:
        """Get current simulation statistics"""
        pass

    def reset_simulation(self, world):
        """Reset the simulation to initial state"""
        pass
```

### Gazebo Robot Interface
```python
from gz.msgs11 import JointCmd, Wrench

class GazeboRobotInterface:
    """
    Interface for controlling robots in Gazebo
    """
    def spawn_robot(self, world, robot_model: str, pose: dict) -> object:
        """Spawn a robot in the simulation"""
        pass

    def set_joint_position(self, robot, joint_name: str, position: float):
        """Set position of a specific joint"""
        pass

    def set_joint_velocity(self, robot, joint_name: str, velocity: float):
        """Set velocity of a specific joint"""
        pass

    def apply_force(self, robot, link_name: str, force: Wrench):
        """Apply force/torque to a specific link"""
        pass

    def get_robot_state(self, robot) -> dict:
        """Get current state of the robot (positions, velocities, etc.)"""
        pass
```

## Unity Simulation Interface Specifications

### Unity Scene Interface
```csharp
using UnityEngine;
using System.Collections.Generic;

public interface IUnitySceneInterface
{
    GameObject CreateObject(string objectType, Vector3 position, Quaternion rotation);
    void ConfigureLighting(LightConfig config);
    void SetRenderPipeline(string pipelineType);
    List<GameObject> GetSceneObjects();
    void UpdateScene(float deltaTime);
    void ResetScene();
}
```

### Unity Robot Interface
```csharp
using UnityEngine;

public interface IUnityRobotInterface
{
    void SetJointPosition(string jointName, float position);
    void SetJointVelocity(string jointName, float velocity);
    Vector3 GetEndEffectorPosition();
    void ApplyForce(string linkName, Vector3 force);
    RobotState GetRobotState();
    void ExecuteTrajectory(RobotTrajectory trajectory);
}
```

## Sensor Simulation Interface Specifications

### Generic Sensor Interface
```python
from sensor_msgs.msg import LaserScan, Imu, Image
from geometry_msgs.msg import PointStamped

class SensorSimulationInterface:
    """
    Interface for sensor simulation in both Gazebo and Unity
    """
    def create_sensor(self, sensor_type: str, config: dict) -> object:
        """Create a sensor simulation"""
        pass

    def configure_noise_model(self, sensor, noise_params: dict):
        """Configure noise characteristics for the sensor"""
        pass

    def get_sensor_data(self, sensor) -> object:
        """Get data from the sensor simulation"""
        pass

    def simulate_sensor_physics(self, sensor, environment_state: dict):
        """Simulate sensor physics based on environment"""
        pass

    def validate_sensor_data(self, data) -> bool:
        """Validate sensor data quality"""
        pass
```

### LIDAR Sensor Interface
```python
from sensor_msgs.msg import LaserScan

class LIDARSensorInterface(SensorSimulationInterface):
    """
    Interface for LIDAR sensor simulation
    """
    def set_scan_parameters(self, sensor, range_min: float, range_max: float,
                           angle_min: float, angle_max: float, increment: float):
        """Configure LIDAR scan parameters"""
        pass

    def simulate_3d_lidar(self, sensor, pointcloud: object) -> LaserScan:
        """Simulate 3D LIDAR from point cloud data"""
        pass

    def add_scan_noise(self, scan: LaserScan, noise_params: dict) -> LaserScan:
        """Add realistic noise to scan data"""
        pass
```

### IMU Sensor Interface
```python
from sensor_msgs.msg import Imu

class IMUSensorInterface(SensorSimulationInterface):
    """
    Interface for IMU sensor simulation
    """
    def simulate_linear_acceleration(self, sensor, gravity: list, acceleration: list) -> list:
        """Simulate linear acceleration readings"""
        pass

    def simulate_angular_velocity(self, sensor, angular_velocity: list) -> list:
        """Simulate angular velocity readings"""
        pass

    def simulate_orientation(self, sensor, orientation: list) -> list:
        """Simulate orientation readings"""
        pass

    def apply_imu_bias(self, imu_data: Imu, bias_params: dict) -> Imu:
        """Apply bias and drift to IMU data"""
        pass
```

## Reinforcement Learning Interface Specifications

### RL Environment Interface
```python
import gymnasium as gym
import numpy as np
from typing import Tuple, Dict, Any

class RLEnvironmentInterface(gym.Env):
    """
    Interface for RL environments in robotics
    """
    def __init__(self, config: dict):
        """Initialize the RL environment"""
        pass

    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Reset the environment to initial state"""
        pass

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """Execute one step in the environment"""
        pass

    def render(self, mode='human') -> Any:
        """Render the environment"""
        pass

    def get_observation_space(self) -> gym.spaces.Space:
        """Get the observation space definition"""
        pass

    def get_action_space(self) -> gym.spaces.Space:
        """Get the action space definition"""
        pass

    def calculate_reward(self, state: np.ndarray, action: np.ndarray,
                        next_state: np.ndarray) -> float:
        """Calculate reward for the transition"""
        pass
```

### RL Agent Interface
```python
import torch
import numpy as np

class RLAgentInterface:
    """
    Interface for RL agents controlling robots
    """
    def __init__(self, observation_space, action_space, config: dict):
        """Initialize the RL agent"""
        pass

    def predict(self, observation: np.ndarray, deterministic: bool = True) -> Tuple[np.ndarray, Any]:
        """Get action prediction from the agent"""
        pass

    def learn(self, total_timesteps: int):
        """Train the agent for specified timesteps"""
        pass

    def save(self, path: str):
        """Save the trained agent"""
        pass

    def load(self, path: str) -> object:
        """Load a trained agent"""
        pass

    def set_training_mode(self, training: bool):
        """Set training or evaluation mode"""
        pass
```

### Robot Control Interface
```python
from geometry_msgs.msg import Pose, Twist
import numpy as np

class RobotControlInterface:
    """
    Interface for robot control in RL contexts
    """
    def execute_action(self, action: np.ndarray) -> bool:
        """Execute an action in the robot environment"""
        pass

    def get_robot_pose(self) -> Pose:
        """Get current robot pose"""
        pass

    def get_robot_twist(self) -> Twist:
        """Get current robot velocity"""
        pass

    def plan_trajectory(self, goal_pose: Pose) -> list:
        """Plan a trajectory to the goal pose"""
        pass

    def execute_trajectory(self, trajectory: list) -> bool:
        """Execute a planned trajectory"""
        pass

    def check_collision(self, trajectory: list) -> bool:
        """Check if trajectory has collisions"""
        pass
```

## Digital Twin Integration Interface

### Digital Twin Manager Interface
```python
class DigitalTwinManagerInterface:
    """
    Interface for managing the digital twin system
    """
    def synchronize_state(self, real_robot_state: dict, sim_robot_state: dict) -> dict:
        """Synchronize state between real and simulated robots"""
        pass

    def calibrate_sensors(self, real_data: object, sim_data: object) -> dict:
        """Calibrate simulation to match real sensor data"""
        pass

    def update_simulation_model(self, real_robot_params: dict) -> bool:
        """Update simulation model based on real robot parameters"""
        pass

    def validate_digital_twin_accuracy(self, real_trajectory: list, sim_trajectory: list) -> float:
        """Validate accuracy of digital twin simulation"""
        pass

    def switch_control_mode(self, mode: str) -> bool:  # "real", "sim", "blended"
        """Switch between real robot, simulation, or blended control"""
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
  "simulationPlatform": ["enum (GAZEBO|UNITY|BOTH)"],
  "gazeboRequired": "boolean",
  "unityRequired": "boolean",
  "rlComponent": "boolean",
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (DRAFT|REVIEW|PUBLISHED)"
}
```

### Gazebo Environment Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "name": "string",
  "description": "string",
  "worldFile": "string",
  "physicsEngine": "enum (ODE|BULLET|SIMBODY)",
  "gravity": "object (x, y, z vector)",
  "objects": ["object"],
  "plugins": ["string"],
  "exampleCode": "string (Python/C++ code)",
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (ACTIVE|ARCHIVED)"
}
```

### Unity Scene Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "name": "string",
  "description": "string",
  "sceneFile": "string",
  "renderPipeline": "enum (BIRP|URP|HDRP)",
  "lightingMode": "enum (BAKED|REALTIME|MIXED)",
  "gameObjects": ["object"],
  "physicsSettings": "object",
  "exampleCode": "string (C# code)",
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (ACTIVE|ARCHIVED)"
}
```

### Sensor Simulation Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "name": "string",
  "type": "enum (LIDAR|DEPTH_CAMERA|IMU|CAMERA|OTHER)",
  "simulator": "enum (GAZEBO|UNITY|BOTH)",
  "parameters": "object",
  "noiseModel": "string",
  "outputFormat": "string",
  "rosMessageType": "string",
  "exampleCode": "string (Python code)",
  "simulationConfig": "object",
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (ACTIVE|ARCHIVED)"
}
```

### RL Environment Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "name": "string",
  "type": "enum (GYM_ENV|CUSTOM_ENV|ROBOT_ENV)",
  "observationSpace": "object",
  "actionSpace": "object",
  "rewardFunction": "string",
  "episodeLength": "integer",
  "simulationBackend": "enum (GAZEBO|UNITY|BOTH)",
  "robotModel": "string",
  "exampleCode": "string (Python code)",
  "trainingConfig": "object",
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (ACTIVE|ARCHIVED)"
}
```

## Error Handling Patterns

### Simulation Error Interface
```python
class SimulationError(Exception):
    """Base exception for simulation operations"""
    def __init__(self, simulation_phase: str, message: str, error_code: str = None):
        self.simulation_phase = simulation_phase  # physics, rendering, sensor, etc.
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)

# Example usage:
try:
    # Simulation operation
    pass
except SimulationError as e:
    node.get_logger().error(f'Simulation Error ({e.simulation_phase}): {e.message} (Code: {e.error_code})')
```

### RL Training Error Interface
```python
class RLTrainingError(Exception):
    """Base exception for RL training operations"""
    def __init__(self, training_phase: str, message: str, error_code: str = None):
        self.training_phase = training_phase  # initialization, training, evaluation, etc.
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)

# Example usage:
try:
    # RL training operation
    pass
except RLTrainingError as e:
    node.get_logger().error(f'RL Training Error ({e.training_phase}): {e.message} (Code: {e.error_code})')
```

## Validation Interfaces

### Gazebo Environment Validation Interface
```python
class GazeboEnvironmentValidatorInterface:
    """
    Interface for validating Gazebo environments
    """
    def validate_world_file(self, world_path: str) -> dict:
        """Validate Gazebo world file syntax and content"""
        pass

    def check_physics_parameters(self, params: dict) -> dict:
        """Validate physics engine parameters"""
        pass

    def validate_model_compatibility(self, model_path: str) -> dict:
        """Validate that model is compatible with Gazebo"""
        pass

    def test_simulation_stability(self, world_config: dict) -> dict:
        """Test simulation stability with given configuration"""
        pass
```

### RL Environment Validation Interface
```python
class RLEnvironmentValidatorInterface:
    """
    Interface for validating RL environments
    """
    def validate_observation_space(self, space: object) -> bool:
        """Validate that observation space is properly defined"""
        pass

    def validate_action_space(self, space: object) -> bool:
        """Validate that action space is properly defined"""
        pass

    def check_reward_function(self, env_function) -> dict:
        """Validate reward function properties"""
        pass

    def test_environment_dynamics(self, env) -> dict:
        """Test that environment dynamics work correctly"""
        pass
```

### Digital Twin Accuracy Interface
```python
class DigitalTwinAccuracyInterface:
    """
    Interface for validating digital twin accuracy
    """
    def compare_real_vs_sim(self, real_data: object, sim_data: object) -> dict:
        """Compare real and simulated data to assess accuracy"""
        pass

    def validate_sensor_models(self, sensor_pairs: list) -> dict:
        """Validate that simulated sensors match real sensors"""
        pass

    def assess_control_transfer(self, real_traj: list, sim_traj: list) -> dict:
        """Assess how well control transfers from sim to real"""
        pass

    def validate_dynamics_model(self, real_robot: object, sim_robot: object) -> dict:
        """Validate that simulated dynamics match real robot"""
        pass
```