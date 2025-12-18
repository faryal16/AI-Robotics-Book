# Data Model: Module 3: The AI-Robot Brain (NVIDIA Isaac™)

## Educational Content Entities

### Module
- **id**: string (unique identifier: "003-isaac-ai-brain")
- **title**: string (main title of the module)
- **description**: string (brief description)
- **targetAudience**: array of strings (robotics researchers, ML engineers, developers transitioning from software AI)
- **prerequisites**: array of strings (Module 1: ROS 2 fundamentals, Module 2: Sensors & Actuators, NVIDIA GPU setup)
- **learningObjectives**: array of strings (Isaac Sim usage, synthetic data generation, VSLAM, humanoid navigation)
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
- **technologyFocus**: string (Isaac Sim, Isaac ROS, Nav2, etc.)
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
- **simulationRequired**: boolean (whether tutorial requires Isaac Sim)
- **gpuRequired**: boolean (whether tutorial requires GPU)
- **aiTraining**: boolean (whether tutorial involves AI model training)
- **createdAt**: datetime
- **updatedAt**: datetime
- **status**: enum (DRAFT | REVIEW | PUBLISHED)

## Isaac Ecosystem Entities

### IsaacSimScene
- **id**: string (unique identifier for documentation purposes)
- **name**: string (scene name)
- **description**: string (what the scene represents)
- **environmentType**: enum (INDOOR | OUTDOOR | LAB | WAREHOUSE | OTHER)
- **objects**: array of IsaacSimObject objects
- **lightingConditions**: array of IsaacSimLighting objects
- **sensorConfigurations**: array of IsaacSimSensor objects
- **physicsSettings**: object (simulation physics parameters)
- **renderingSettings**: object (rendering quality parameters)
- **createdAt**: datetime
- **updatedAt**: datetime

### IsaacSimObject
- **id**: string (unique identifier for documentation purposes)
- **name**: string (object name)
- **type**: enum (ROBOT | OBSTACLE | FURNITURE | HUMANOID | OTHER)
- **geometry**: string (mesh file path or primitive type)
- **material**: string (material properties)
- **pose**: object (position and orientation in 3D space)
- **physicsProperties**: object (mass, friction, etc.)
- **createdAt**: datetime
- **updatedAt**: datetime

### IsaacSimSensor
- **id**: string (unique identifier for documentation purposes)
- **sensorType**: enum (RGB_CAMERA | DEPTH_CAMERA | LIDAR | IMU | OTHER)
- **name**: string (sensor name)
- **position**: object (position relative to robot)
- **orientation**: object (orientation relative to robot)
- **parameters**: object (sensor-specific parameters)
- **outputTopics**: array of string (ROS topics for sensor data)
- **createdAt**: datetime
- **updatedAt**: datetime

### IsaacSimLighting
- **id**: string (unique identifier for documentation purposes)
- **name**: string (lighting setup name)
- **type**: enum (DIRECTIONAL | POINT | SPOT | AMBIENT | HDRI)
- **intensity**: float (light intensity)
- **color**: object (RGB color values)
- **position**: object (position in 3D space)
- **createdAt**: datetime
- **updatedAt**: datetime

## AI and Navigation Entities

### PerceptionModel
- **id**: string (unique identifier for documentation purposes)
- **name**: string (model name)
- **type**: enum (CLASSIFICATION | DETECTION | SEGMENTATION | VSLAM | OTHER)
- **architecture**: string (model architecture, e.g., ResNet, YOLO)
- **inputType**: string (expected input format)
- **outputType**: string (expected output format)
- **trainingData**: string (source of training data)
- **syntheticDataRatio**: float (percentage of synthetic data used)
- **accuracyMetrics**: object (model performance metrics)
- **createdAt**: datetime
- **updatedAt**: datetime

### TrainingDataset
- **id**: string (unique identifier for documentation purposes)
- **name**: string (dataset name)
- **type**: enum (SYNTHETIC | REAL | MIXED)
- **size**: integer (number of samples)
- **contentDescription**: string (what the dataset contains)
- **syntheticGenerationParams**: object (parameters used for synthetic generation)
- **labelAccuracy**: float (accuracy of labels)
- **createdAt**: datetime
- **updatedAt**: datetime

### NavigationConfig
- **id**: string (unique identifier for documentation purposes)
- **name**: string (configuration name)
- **robotType**: enum (WHEELED | BIPEDAL | QUADRUPED | OTHER)
- **plannerType**: enum (GLOBAL_PLANNER | LOCAL_PLANNER | CONTROLLER)
- **parameters**: object (planner-specific parameters)
- **mapType**: enum (TOPOLOGICAL | GRID | MESH | OTHER)
- **obstacleHandling**: string (how obstacles are handled)
- **createdAt**: datetime
- **updatedAt**: datetime

### PathPlan
- **id**: string (unique identifier for documentation purposes)
- **name**: string (plan name)
- **startPose**: object (starting position and orientation)
- **goalPose**: object (goal position and orientation)
- **waypoints**: array of object (intermediate waypoints)
- **computedPath**: array of object (actual computed path points)
- **planningTime**: float (time taken to compute path in seconds)
- **pathLength**: float (length of path in meters)
- **safetyMargin**: float (minimum distance to obstacles)
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

### IsaacSimScene Validation
- Name must be unique within the module
- Must have at least one object
- Physics settings must be valid
- Rendering settings must be within reasonable bounds

### PerceptionModel Validation
- Name must be unique within the module
- Type must be one of the defined enum values
- Accuracy metrics must be within valid ranges (0.0-1.0 for percentages)

### PathPlan Validation
- Start and goal poses must be defined
- Waypoints must be valid 3D coordinates
- Planning time must be positive
- Path length must be positive

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
- Module has many IsaacSimScene examples
- IsaacSimScene contains many IsaacSimObject
- IsaacSimScene contains many IsaacSimSensor
- IsaacSimScene contains many IsaacSimLighting
- Tutorial may use many IsaacSimScene
- TrainingDataset may be used by many PerceptionModel
- NavigationConfig may be used by many PathPlan
- PerceptionModel may be used in many Tutorials