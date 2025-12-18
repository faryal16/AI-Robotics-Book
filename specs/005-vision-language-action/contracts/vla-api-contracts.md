# API Contracts: Module 5: Vision-Language-Action

## Overview
This document defines the conceptual API contracts and interfaces for the Vision-Language-Action learning module. These represent the interfaces for voice processing, language understanding, vision recognition, and action execution that students will learn to use and implement.

## Voice Processing Interface Specifications

### Voice Command Interface
```python
from std_msgs.msg import String
import speech_recognition as sr

class VoiceCommandInterface:
    """
    Interface for voice command processing
    """
    def start_listening(self):
        """Start listening for voice commands"""
        pass

    def stop_listening(self):
        """Stop listening for voice commands"""
        pass

    def set_language(self, language: str):
        """Set the recognition language"""
        pass

    def get_transcription(self, audio_data) -> str:
        """Get text transcription from audio data"""
        pass

    def validate_audio_input(self, audio_source) -> bool:
        """Validate that audio source is working properly"""
        pass

    def adjust_for_noise(self, audio_source):
        """Adjust recognition for ambient noise"""
        pass
```

### Whisper Integration Interface
```python
import whisper
import torch

class WhisperInterface:
    """
    Interface for OpenAI Whisper model integration
    """
    def load_model(self, model_name: str) -> object:
        """Load Whisper model (tiny, base, small, medium, large)"""
        pass

    def transcribe_audio(self, audio_file_path: str, language: str = None) -> dict:
        """Transcribe audio file using Whisper"""
        pass

    def transcribe_microphone(self, duration: int = None) -> dict:
        """Transcribe live microphone input"""
        pass

    def get_available_models(self) -> list:
        """Get list of available Whisper models"""
        pass

    def process_audio_batch(self, audio_files: list) -> list:
        """Process multiple audio files in batch"""
        pass
```

## Language Processing Interface Specifications

### LLM Interface
```python
import openai
from typing import Dict, List, Any

class LLMInterface:
    """
    Interface for Large Language Model integration
    """
    def __init__(self, api_key: str = None, model_name: str = "gpt-3.5-turbo"):
        """Initialize LLM with API key and model"""
        pass

    def generate_text(self, prompt: str, max_tokens: int = 200) -> str:
        """Generate text response from prompt"""
        pass

    def chat_completion(self, messages: List[Dict[str, str]],
                       temperature: float = 0.7) -> str:
        """Generate chat completion from message history"""
        pass

    def extract_structured_data(self, text: str, schema: Dict) -> Dict:
        """Extract structured data from text according to schema"""
        pass

    def classify_intent(self, text: str, possible_intents: List[str]) -> str:
        """Classify the intent of the given text"""
        pass

    def generate_action_sequence(self, natural_language_command: str) -> List[Dict]:
        """Generate sequence of actions from natural language command"""
        pass
```

### Cognitive Planning Interface
```python
from typing import Dict, List, Any

class CognitivePlanningInterface:
    """
    Interface for cognitive planning using LLMs
    """
    def create_task_plan(self, high_level_task: str) -> Dict:
        """Create a detailed plan for executing a high-level task"""
        pass

    def decompose_task(self, task: str) -> List[str]:
        """Decompose a complex task into simpler subtasks"""
        pass

    def validate_plan_feasibility(self, plan: List[Dict], robot_capabilities: List[str]) -> Dict:
        """Validate if the plan is feasible given robot capabilities"""
        pass

    def handle_plan_execution_errors(self, error_description: str, current_plan: List[Dict]) -> Dict:
        """Generate recovery plan when execution fails"""
        pass

    def optimize_plan(self, plan: List[Dict], constraints: Dict) -> List[Dict]:
        """Optimize plan based on given constraints (time, energy, etc.)"""
        pass
```

## Vision Processing Interface Specifications

### Vision Recognition Interface
```python
from sensor_msgs.msg import Image
from typing import List, Dict, Any

class VisionRecognitionInterface:
    """
    Interface for vision-based object recognition
    """
    def load_detection_model(self, model_path: str):
        """Load object detection model"""
        pass

    def detect_objects(self, image: Image) -> List[Dict]:
        """Detect objects in the given image"""
        pass

    def recognize_object(self, image: Image, object_name: str) -> Dict:
        """Recognize a specific object in the image"""
        pass

    def track_object(self, initial_bbox: List[int], video_stream) -> List[List[int]]:
        """Track object across video frames"""
        pass

    def estimate_pose(self, object_name: str, image: Image) -> Dict:
        """Estimate 3D pose of object in image"""
        pass

    def segment_image(self, image: Image) -> object:  # Segmentation mask
        """Perform semantic segmentation on image"""
        pass
```

### Vision Pipeline Interface
```python
class VisionPipelineInterface:
    """
    Interface for complete vision processing pipeline
    """
    def add_preprocessing_step(self, step_function):
        """Add a preprocessing step to the pipeline"""
        pass

    def add_detection_step(self, detector_function):
        """Add an object detection step to the pipeline"""
        pass

    def add_postprocessing_step(self, step_function):
        """Add a postprocessing step to the pipeline"""
        pass

    def process_frame(self, image: object) -> Dict:
        """Process a single image frame through the pipeline"""
        pass

    def process_video_stream(self, stream) -> List[Dict]:
        """Process a video stream through the pipeline"""
        pass

    def get_pipeline_performance(self) -> Dict:
        """Get performance metrics for the pipeline"""
        pass
```

## Action Execution Interface Specifications

### ROS2 Action Interface
```python
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from typing import Dict, Any

class ROS2ActionInterface:
    """
    Interface for ROS2 action execution
    """
    def send_goal(self, action_name: str, goal: Any) -> str:
        """Send a goal to an action server"""
        pass

    def wait_for_result(self, goal_id: str, timeout_sec: float) -> Any:
        """Wait for action result with timeout"""
        pass

    def cancel_goal(self, goal_id: str):
        """Cancel a running goal"""
        pass

    def get_action_status(self, goal_id: str) -> GoalStatus:
        """Get current status of an action"""
        pass

    def execute_action_sequence(self, actions: List[Dict]) -> bool:
        """Execute a sequence of actions"""
        pass
```

### Manipulation Interface
```python
from geometry_msgs.msg import Pose, Point
from typing import Dict, List

class ManipulationInterface:
    """
    Interface for robot manipulation tasks
    """
    def move_to_pose(self, pose: Pose) -> bool:
        """Move end-effector to specified pose"""
        pass

    def grasp_object(self, object_pose: Pose, grasp_type: str = "parallel") -> bool:
        """Grasp an object at the specified pose"""
        pass

    def release_object(self) -> bool:
        """Release currently grasped object"""
        pass

    def plan_grasp(self, object_info: Dict) -> Dict:
        """Plan grasp approach for an object"""
        pass

    def execute_trajectory(self, waypoints: List[Pose]) -> bool:
        """Execute a trajectory defined by waypoints"""
        pass

    def check_collision(self, trajectory: List[Pose]) -> bool:
        """Check if trajectory has collisions"""
        pass
```

## Integration Interface Specifications

### Multimodal Fusion Interface
```python
from typing import Dict, Any, List

class MultimodalFusionInterface:
    """
    Interface for fusing information from multiple modalities
    """
    def fuse_voice_and_vision(self, voice_command: str, vision_data: List[Dict]) -> Dict:
        """Fuse voice command with visual information"""
        pass

    def reconcile_multimodal_inputs(self, inputs: Dict[str, Any]) -> Dict:
        """Reconcile potentially conflicting multimodal inputs"""
        pass

    def maintain_world_model(self, new_observations: List[Dict]) -> Dict:
        """Update world model with new observations from all modalities"""
        pass

    def generate_context_aware_response(self, multimodal_input: Dict) -> str:
        """Generate response considering all modalities and context"""
        pass

    def handle_modality_failure(self, failed_modality: str, fallback_strategy: str) -> bool:
        """Handle failure in one modality using fallback strategies"""
        pass
```

### Humanoid Control Interface
```python
class HumanoidControlInterface:
    """
    Interface for controlling humanoid robots
    """
    def set_joint_positions(self, joint_positions: Dict[str, float]) -> bool:
        """Set positions for multiple joints"""
        pass

    def execute_behavior(self, behavior_name: str) -> bool:
        """Execute a predefined humanoid behavior"""
        pass

    def maintain_balance(self, external_forces: List[float]) -> bool:
        """Maintain balance when external forces are applied"""
        pass

    def perform_bipedal_locomotion(self, target_location: List[float]) -> bool:
        """Perform bipedal walking to target location"""
        pass

    def coordinate_arm_and_leg_movement(self, task: str) -> bool:
        """Coordinate arm and leg movements for complex tasks"""
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
  "modalities": ["enum (VOICE|LANGUAGE|VISION|ACTION)"],
  "voiceComponent": "boolean",
  "languageComponent": "boolean",
  "visionComponent": "boolean",
  "actionComponent": "boolean",
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (DRAFT|REVIEW|PUBLISHED)"
}
```

### Voice Command Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "transcription": "string",
  "confidence": "float (0.0-1.0)",
  "language": "string",
  "intent": "string",
  "entities": ["object"],
  "expectedActions": ["string"],
  "exampleAudio": "string (path to audio file)",
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (ACTIVE|ARCHIVED)"
}
```

### Cognitive Plan Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "taskId": "string",
  "generatedSteps": ["object"],
  "confidence": "float (0.0-1.0)",
  "reasoningTrace": "string",
  "constraints": ["string"],
  "successCriteria": "string",
  "recoveryStrategies": ["string"],
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (ACTIVE|ARCHIVED)"
}
```

### Vision Recognition Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "objectName": "string",
  "detectionAccuracy": "float (0.0-1.0)",
  "modelType": "string",
  "inputResolution": "object (width, height)",
  "supportedEnvironments": ["string"],
  "exampleImages": ["string"],
  "trainingRequirements": "string",
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (ACTIVE|ARCHIVED)"
}
```

### Action Sequence Interface
```json
{
  "id": "string",
  "moduleId": "string",
  "name": "string",
  "description": "string",
  "steps": ["object"],
  "priority": "integer",
  "timeout": "float (seconds)",
  "successCriteria": "string",
  "recoveryActions": ["string"],
  "requiredCapabilities": ["string"],
  "createdAt": "datetime",
  "updatedAt": "datetime",
  "status": "enum (ACTIVE|ARCHIVED)"
}
```

## Error Handling Patterns

### Voice Processing Error Interface
```python
class VoiceProcessingError(Exception):
    """Base exception for voice processing operations"""
    def __init__(self, processing_stage: str, message: str, error_code: str = None):
        self.processing_stage = processing_stage  # recognition, transcription, etc.
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)

# Example usage:
try:
    # Voice processing operation
    pass
except VoiceProcessingError as e:
    node.get_logger().error(f'Voice Processing Error ({e.processing_stage}): {e.message} (Code: {e.error_code})')
```

### Multimodal Integration Error Interface
```python
class MultimodalIntegrationError(Exception):
    """Base exception for multimodal integration operations"""
    def __init__(self, modalities_involved: list, message: str, error_code: str = None):
        self.modalities_involved = modalities_involved  # e.g., ["voice", "vision"]
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)

# Example usage:
try:
    # Multimodal operation
    pass
except MultimodalIntegrationError as e:
    node.get_logger().error(f'Multimodal Error (involving {e.modalities_involved}): {e.message} (Code: {e.error_code})')
```

## Validation Interfaces

### Voice Command Validation Interface
```python
class VoiceCommandValidatorInterface:
    """
    Interface for validating voice commands
    """
    def validate_transcription_quality(self, transcription: str, confidence: float) -> dict:
        """Validate quality of voice transcription"""
        pass

    def check_command_syntax(self, command: str) -> dict:
        """Check if command follows expected syntax patterns"""
        pass

    def verify_command_feasibility(self, command: str, robot_capabilities: list) -> dict:
        """Verify if command is feasible given robot capabilities"""
        pass

    def assess_language_complexity(self, command: str) -> dict:
        """Assess complexity of natural language command"""
        pass
```

### Multimodal Pipeline Validation Interface
```python
class MultimodalPipelineValidatorInterface:
    """
    Interface for validating multimodal pipelines
    """
    def validate_modality_synchronization(self, timestamps: dict) -> dict:
        """Validate synchronization between different modalities"""
        pass

    def check_data_flow_integrity(self, pipeline_config: dict) -> dict:
        """Check integrity of data flow through the pipeline"""
        pass

    def assess_real_time_performance(self, pipeline) -> dict:
        """Assess if pipeline meets real-time performance requirements"""
        pass

    def validate_error_handling_mechanisms(self, pipeline_config: dict) -> dict:
        """Validate error handling in the multimodal pipeline"""
        pass
```

### Action Plan Validation Interface
```python
class ActionPlanValidatorInterface:
    """
    Interface for validating action plans
    """
    def validate_plan_syntax(self, plan: list) -> dict:
        """Validate syntax of action plan"""
        pass

    def check_plan_feasibility(self, plan: list, robot_state: dict) -> dict:
        """Check if plan is feasible given current robot state"""
        pass

    def verify_plan_safety(self, plan: list, environment_model: dict) -> dict:
        """Verify that plan is safe to execute"""
        pass

    def assess_plan_efficiency(self, plan: list) -> dict:
        """Assess efficiency of the action plan"""
        pass
```