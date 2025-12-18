# Data Model: Module 5: Vision-Language-Action

## Educational Content Entities

### Module
- **id**: string (unique identifier: "005-vision-language-action")
- **title**: string (main title of the module)
- **description**: string (brief description)
- **targetAudience**: array of strings (robotics researchers, ML engineers, developers transitioning from software AI)
- **prerequisites**: array of strings (Module 1-4: Complete series, basic ML concepts, Python programming)
- **learningObjectives**: array of strings (voice processing, LLM cognitive planning, vision recognition, multimodal integration)
- **duration**: integer (estimated hours to complete)
- **createdAt**: datetime
- **updatedAt**: datetime
- **status**: enum (DRAFT | REVIEW | PUBLISHED)

### Chapter
- **id**: string (unique identifier)
- **moduleId**: string (reference to Module)
- **title**: string (chapter title)
- **subtitle**: string (optional subtitle)
- **content**: string (main content in Markdown)
- **order**: integer (position in module)
- **difficulty**: enum (BEGINNER | INTERMEDIATE | ADVANCED)
- **estimatedTime**: integer (minutes to complete)
- **modalityFocus**: string (VOICE | LANGUAGE | VISION | ACTION | INTEGRATION)
- **createdAt**: datetime
- **updatedAt**: datetime
- **status**: enum (DRAFT | REVIEW | PUBLISHED)
- **frontmatter**: object (Docusaurus frontmatter)

### Tutorial
- **id**: string (unique identifier)
- **moduleId**: string (reference to Module)
- **title**: string (tutorial title)
- **description**: string (brief description)
- **files**: array of string (paths to tutorial files)
- **dependencies**: array of string (required packages/tools)
- **instructions**: string (step-by-step instructions in Markdown)
- **expectedOutcome**: string (what learner should achieve)
- **voiceComponent**: boolean (whether tutorial involves voice processing)
- **llmComponent**: boolean (whether tutorial involves LLM planning)
- **visionComponent**: boolean (whether tutorial involves vision processing)
- **actionComponent**: boolean (whether tutorial involves action execution)
- **createdAt**: datetime
- **updatedAt**: datetime
- **status**: enum (DRAFT | REVIEW | PUBLISHED)

## Voice Processing Entities

### VoiceCommand
- **id**: string (unique identifier for documentation purposes)
- **transcription**: string (text transcription of the voice command)
- **confidence**: float (confidence score of transcription)
- **language**: string (detected language)
- **timestamp**: datetime (when command was received)
- **intent**: string (parsed intent from the command)
- **entities**: array of object (extracted entities like objects, locations)
- **createdAt**: datetime
- **updatedAt**: datetime

### WhisperModel
- **id**: string (unique identifier for documentation purposes)
- **name**: string (model name, e.g., "base", "small", "medium", "large")
- **languageSupport**: array of string (languages the model supports)
- **accuracyMetrics**: object (WER, CER, and other accuracy metrics)
- **computationalRequirements**: object (CPU/GPU requirements)
- **latencyCharacteristics**: object (processing time characteristics)
- **createdAt**: datetime
- **updatedAt**: datetime

### VoiceProcessor
- **id**: string (unique identifier for documentation purposes)
- **modelId**: string (reference to WhisperModel)
- **audioInputSource**: string (microphone, file, stream)
- **processingPipeline**: object (preprocessing, model, postprocessing steps)
- **noiseReductionEnabled**: boolean (whether noise reduction is applied)
- **languageDetectionEnabled**: boolean (whether language detection is used)
- **createdAt**: datetime
- **updatedAt**: datetime

## Language Processing Entities

### LLMModel
- **id**: string (unique identifier for documentation purposes)
- **name**: string (model name, e.g., "gpt-4", "llama-2", etc.)
- **provider**: string (OpenAI, Hugging Face, etc.)
- **contextWindow**: integer (maximum context length in tokens)
- **capabilities**: array of string (reasoning, planning, generation, etc.)
- **apiEndpoint**: string (API endpoint if applicable)
- **createdAt**: datetime
- **updatedAt**: datetime

### CognitivePlan
- **id**: string (unique identifier for documentation purposes)
- **taskId**: string (high-level task description)
- **generatedSteps**: array of PlanStep objects
- **confidence**: float (confidence in the plan)
- **reasoningTrace**: string (explanation of how the plan was generated)
- **constraints**: array of string (constraints considered in planning)
- **createdAt**: datetime
- **updatedAt**: datetime

### PlanStep
- **id**: string (unique identifier for documentation purposes)
- **description**: string (human-readable description of the step)
- **actionType**: enum (NAVIGATION | MANIPULATION | PERCEPTION | COMMUNICATION)
- **parameters**: object (parameters for the action)
- **dependencies**: array of string (other step IDs this step depends on)
- **successCriteria**: string (how to determine if step was successful)
- **createdAt**: datetime
- **updatedAt**: datetime

### TaskSpecification
- **id**: string (unique identifier for documentation purposes)
- **commandText**: string (original natural language command)
- **structuredTask**: object (structured representation of the task)
- **requiredCapabilities**: array of string (capabilities needed to complete task)
- **expectedObjects**: array of string (objects expected in environment)
- **createdAt**: datetime
- **updatedAt**: datetime

## Vision Processing Entities

### VisionSystem
- **id**: string (unique identifier for documentation purposes)
- **sensorType**: enum (RGB_CAMERA | DEPTH_CAMERA | RGBD | MONOCULAR)
- **processingPipeline**: object (detection, recognition, tracking components)
- **supportedObjects**: array of string (object types the system can recognize)
- **accuracyMetrics**: object (precision, recall, mAP, etc.)
- **processingRate**: float (frames per second)
- **createdAt**: datetime
- **updatedAt**: datetime

### DetectedObject
- **id**: string (unique identifier for documentation purposes)
- **name**: string (object class name)
- **confidence**: float (detection confidence)
- **boundingBox**: object (coordinates of bounding box)
- **pose**: object (3D position and orientation if available)
- **mask**: object (segmentation mask if available)
- **timestamp**: datetime (when object was detected)
- **createdAt**: datetime
- **updatedAt**: datetime

### VisionModel
- **id**: string (unique identifier for documentation purposes)
- **name**: string (model name, e.g., "YOLOv8", "Mask R-CNN", etc.)
- **architecture**: string (model architecture)
- **inputResolution**: object (required input dimensions)
- **supportedTasks**: array of string (detection, segmentation, classification)
- **createdAt**: datetime
- **updatedAt**: datetime

## Action Execution Entities

### ROS2Action
- **id**: string (unique identifier for documentation purposes)
- **name**: string (action name)
- **type**: string (action type, e.g., "control_msgs/FollowJointTrajectory")
- **description**: string (what the action does)
- **goalStructure**: object (definition of goal message)
- **feedbackStructure**: object (definition of feedback message)
- **resultStructure**: object (definition of result message)
- **createdAt**: datetime
- **updatedAt**: datetime

### ActionSequence
- **id**: string (unique identifier for documentation purposes)
- **name**: string (sequence name)
- **steps**: array of ActionStep objects
- **priority**: integer (execution priority)
- **preemptionAllowed**: boolean (whether sequence can be preempted)
- **createdAt**: datetime
- **updatedAt**: datetime

### ActionStep
- **id**: string (unique identifier for documentation purposes)
- **actionId**: string (reference to ROS2Action)
- **parameters**: object (action-specific parameters)
- **timeout**: float (maximum time to wait for completion)
- **successCondition**: string (condition to check for success)
- **recoveryActions**: array of string (actions to try if this fails)
- **createdAt**: datetime
- **updatedAt**: datetime

### ManipulationTask
- **id**: string (unique identifier for documentation purposes)
- **name**: string (task name)
- **description**: string (task description)
- **requiredObjects**: array of string (objects needed for task)
- **requiredJoints**: array of string (joints needed for manipulation)
- **endEffectorPose**: object (required end-effector pose)
- **graspType**: enum (PREHENSILE | SUCTION | MAGNETIC | OTHER)
- **createdAt**: datetime
- **updatedAt**: datetime

## Integration Entities

### MultimodalPipeline
- **id**: string (unique identifier for documentation purposes)
- **name**: string (pipeline name)
- **components**: array of string (voice, language, vision, action components)
- **executionFlow**: object (how components interact)
- **synchronizationMechanism**: string (how modalities are synchronized)
- **errorHandlingStrategy**: string (how errors are handled across modalities)
- **createdAt**: datetime
- **updatedAt**: datetime

### HumanoidRobot
- **id**: string (unique identifier for documentation purposes)
- **name**: string (robot name)
- **description**: string (robot description)
- **sensors**: array of string (sensor types available)
- **actuators**: array of string (actuator types available)
- **capabilities**: array of string (robot capabilities)
- **supportedActions**: array of string (actions the robot can perform)
- **createdAt**: datetime
- **updatedAt**: datetime

## Validation Rules

### Module Validation
- Title must be 1-200 characters
- Must have at least one target audience
- Duration must be a positive integer
- Status must be one of the defined enum values

### Chapter Validation
- Title must be 1-200 characters
- Must belong to a valid module
- Order must be a positive integer
- Difficulty must be one of the defined enum values
- Content must be in valid Markdown format

### VoiceCommand Validation
- Transcription must not be empty
- Confidence must be between 0.0 and 1.0
- Timestamp must be valid

### CognitivePlan Validation
- Must have at least one plan step
- Plan steps must form a valid sequence (no circular dependencies)
- Confidence must be between 0.0 and 1.0

### ActionSequence Validation
- Must have at least one action step
- Action steps must be properly defined
- Timeouts must be positive

## State Transitions

### Module States
- DRAFT → REVIEW (when content is complete and ready for review)
- REVIEW → DRAFT (when changes are requested)
- REVIEW → PUBLISHED (when approved for publication)
- PUBLISHED → REVIEW (when updates are needed)

### Chapter States
- DRAFT → REVIEW (when content is complete and ready for review)
- REVIEW → DRAFT (when changes are requested)
- REVIEW → PUBLISHED (when approved for publication)
- PUBLISHED → REVIEW (when updates are needed)

### Tutorial States
- DRAFT → REVIEW (when tutorial is complete and tested)
- REVIEW → DRAFT (when changes are requested)
- REVIEW → PUBLISHED (when approved for publication)
- PUBLISHED → REVIEW (when updates are needed)

## Relationships
- Module has many Chapters
- Module has many Tutorials
- Tutorial may involve many VoiceCommand
- CognitivePlan contains many PlanStep
- ActionSequence contains many ActionStep
- VisionSystem detects many DetectedObject
- MultimodalPipeline integrates many components
- HumanoidRobot performs many ManipulationTask
- TaskSpecification generates many CognitivePlan