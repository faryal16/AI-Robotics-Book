# Data Model: Module 4: The Digital Twin

## Educational Content Entities

### Module
- **id**: string (unique identifier: "004-digital-twin")
- **title**: string (main title of the module)
- **description**: string (brief description)
- **targetAudience**: array of strings (robotics researchers, ML engineers, developers transitioning from software AI)
- **prerequisites**: array of strings (Module 1: ROS 2 fundamentals, Module 2: Sensors & Actuators, basic RL concepts)
- **learningObjectives**: array of strings (physics simulation, Unity rendering, sensor simulation, RL for robot control)
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
- **simulationPlatform**: string (Gazebo, Unity, or Both)
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
- **gazeboRequired**: boolean (whether tutorial requires Gazebo)
- **unityRequired**: boolean (whether tutorial requires Unity)
- **rlComponent**: boolean (whether tutorial involves reinforcement learning)
- **createdAt**: datetime
- **updatedAt**: datetime
- **status**: enum (DRAFT | REVIEW | PUBLISHED)

## Simulation Entities

### GazeboEnvironment
- **id**: string (unique identifier for documentation purposes)
- **name**: string (environment name)
- **description**: string (what the environment represents)
- **worldFile**: string (path to .world file)
- **physicsEngine**: enum (ODE | BULLET | SIMBODY)
- **gravity**: object (gravity vector, default: [0, 0, -9.8])
- **objects**: array of GazeboObject objects
- **lighting**: array of GazeboLight objects
- **plugins**: array of GazeboPlugin objects
- **createdAt**: datetime
- **updatedAt**: datetime

### GazeboObject
- **id**: string (unique identifier for documentation purposes)
- **name**: string (object name)
- **type**: enum (ROBOT | OBSTACLE | FURNITURE | SENSOR | OTHER)
- **modelFile**: string (path to model file or model name)
- **pose**: object (position and orientation in 3D space)
- **physicsProperties**: object (mass, friction, restitution, etc.)
- **visualProperties**: object (color, texture, etc.)
- **collisions**: array of GazeboCollision objects
- **createdAt**: datetime
- **updatedAt**: datetime

### GazeboCollision
- **id**: string (unique identifier for documentation purposes)
- **geometryType**: enum (BOX | CYLINDER | SPHERE | MESH | PLANE)
- **size**: object (dimensions for the geometry)
- **surfaceProperties**: object (friction, bounce, etc.)
- **createdAt**: datetime
- **updatedAt**: datetime

### UnityScene
- **id**: string (unique identifier for documentation purposes)
- **name**: string (scene name)
- **description**: string (what the scene represents)
- **sceneFile**: string (path to .unity file)
- **renderPipeline**: enum (BIRP | URP | HDRP)
- **lightingMode**: enum (BAKED | REALTIME | MIXED)
- **gameObjects**: array of UnityGameObject objects
- **cameras**: array of UnityCamera objects
- **lighting**: array of UnityLight objects
- **physicsSettings**: object (physics parameters)
- **createdAt**: datetime
- **updatedAt**: datetime

### UnityGameObject
- **id**: string (unique identifier for documentation purposes)
- **name**: string (game object name)
- **type**: enum (ROBOT | OBSTACLE | FURNITURE | SENSOR | UI_ELEMENT | OTHER)
- **prefab**: string (path to prefab file)
- **transform**: object (position, rotation, scale)
- **components**: array of string (component names)
- **material**: string (material file path)
- **createdAt**: datetime
- **updatedAt**: datetime

### SensorSimulation
- **id**: string (unique identifier for documentation purposes)
- **name**: string (sensor name)
- **type**: enum (LIDAR | DEPTH_CAMERA | IMU | CAMERA | OTHER)
- **simulator**: enum (GAZEBO | UNITY | BOTH)
- **parameters**: object (sensor-specific parameters)
- **noiseModel**: string (noise characteristics)
- **outputFormat**: string (expected output format)
- **rosMessageType**: string (ROS message type)
- **createdAt**: datetime
- **updatedAt**: datetime

### LIDARSensor
- **id**: string (inherits from SensorSimulation)
- **rangeMin**: float (minimum range in meters)
- **rangeMax**: float (maximum range in meters)
- **angleMin**: float (minimum angle in radians)
- **angleMax**: float (maximum angle in radians)
- **angleIncrement**: float (angle increment in radians)
- **verticalChannels**: integer (number of vertical channels for 3D LIDAR)
- **verticalAngleMin**: float (minimum vertical angle)
- **verticalAngleMax**: float (maximum vertical angle)

### DepthCameraSensor
- **id**: string (inherits from SensorSimulation)
- **imageWidth**: integer (image width in pixels)
- **imageHeight**: integer (image height in pixels)
- **fovHorizontal**: float (horizontal field of view in radians)
- **fovVertical**: float (vertical field of view in radians)
- **depthRangeMin**: float (minimum depth in meters)
- **depthRangeMax**: float (maximum depth in meters)
- **depthPrecision**: float (depth precision in meters)

### IMUSensor
- **id**: string (inherits from SensorSimulation)
- **linearAccelerationNoiseDensity**: float (noise density for linear acceleration)
- **angularVelocityNoiseDensity**: float (noise density for angular velocity)
- **linearAccelerationRandomWalk**: float (random walk for linear acceleration)
- **angularVelocityRandomWalk**: float (random walk for angular velocity)

## Reinforcement Learning Entities

### RLEnvironment
- **id**: string (unique identifier for documentation purposes)
- **name**: string (environment name)
- **type**: enum (GYM_ENV | CUSTOM_ENV | ROBOT_ENV)
- **observationSpace**: object (definition of observation space)
- **actionSpace**: object (definition of action space)
- **rewardFunction**: string (description of reward function)
- **episodeLength**: integer (maximum steps per episode)
- **simulationBackend**: enum (GAZEBO | UNITY | BOTH)
- **robotModel**: string (robot model used in environment)
- **createdAt**: datetime
- **updatedAt**: datetime

### RLEpisode
- **id**: string (unique identifier for documentation purposes)
- **environmentId**: string (reference to RLEnvironment)
- **episodeNumber**: integer (episode number)
- **steps**: integer (number of steps in episode)
- **totalReward**: float (cumulative reward)
- **terminalState**: boolean (whether episode ended in terminal state)
- **success**: boolean (whether episode was successful)
- **createdAt**: datetime
- **updatedAt**: datetime

### RLAgent
- **id**: string (unique identifier for documentation purposes)
- **name**: string (agent name)
- **algorithm**: enum (DQN | PPO | SAC | TD3 | A2C | OTHER)
- **observationSpace**: object (agent's observation space)
- **actionSpace**: object (agent's action space)
- **networkArchitecture**: string (description of neural network)
- **trainingEpisodes**: integer (number of episodes for training)
- **createdAt**: datetime
- **updatedAt**: datetime

### RobotArm
- **id**: string (unique identifier for documentation purposes)
- **name**: string (arm name)
- **type**: enum (2DOF | 3DOF | 6DOF | 7DOF | CUSTOM)
- **links**: array of RobotLink objects
- **joints**: array of RobotJoint objects
- **endEffector**: RobotEndEffector object
- **controlMode**: enum (POSITION | VELOCITY | TORQUE)
- **simulationPlatform**: enum (GAZEBO | UNITY | BOTH)
- **createdAt**: datetime
- **updatedAt**: datetime

### RobotLink
- **id**: string (unique identifier for documentation purposes)
- **name**: string (link name)
- **mass**: float (mass in kg)
- **inertia**: object (inertia tensor)
- **geometry**: object (geometric properties)
- **visualMesh**: string (path to visual mesh)
- **collisionMesh**: string (path to collision mesh)
- **createdAt**: datetime
- **updatedAt**: datetime

### RobotJoint
- **id**: string (unique identifier for documentation purposes)
- **name**: string (joint name)
- **type**: enum (REVOLUTE | PRISMATIC | FIXED | CONTINUOUS)
- **parentLink**: string (name of parent link)
- **childLink**: string (name of child link)
- **axis**: object (rotation/translation axis)
- **limits**: RobotJointLimits object
- **dynamics**: object (damping, friction, etc.)
- **createdAt**: datetime
- **updatedAt**: datetime

### RobotJointLimits
- **effort**: float (maximum effort)
- **velocity**: float (maximum velocity)
- **lower**: float (lower position limit)
- **upper**: float (upper position limit)

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

### GazeboEnvironment Validation
- Name must be unique within the module
- World file must exist and be valid
- Physics properties must be within reasonable bounds

### UnityScene Validation
- Name must be unique within the module
- Scene file must exist and be valid
- Render pipeline must be one of the defined enum values

### SensorSimulation Validation
- Name must be unique within the module
- Type must be one of the defined enum values
- Parameters must be appropriate for sensor type

### RLEnvironment Validation
- Name must be unique within the module
- Observation and action spaces must be properly defined
- Episode length must be positive

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
- Module has many GazeboEnvironment examples
- Module has many UnityScene examples
- Tutorial may use many GazeboEnvironment
- Tutorial may use many UnityScene
- GazeboEnvironment contains many GazeboObject
- UnityScene contains many UnityGameObject
- RLEnvironment uses one RobotArm
- RLEnvironment has many RLEpisode
- RLAgent operates in many RLEnvironment
- RobotArm has many RobotLink
- RobotArm has many RobotJoint
- SensorSimulation may be used in many GazeboEnvironment and UnityScene