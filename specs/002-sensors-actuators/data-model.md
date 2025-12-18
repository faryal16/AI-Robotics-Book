# Data Model: Module 2: Sensors & Actuators

## Educational Content Entities

### Module
- **id**: string (unique identifier: "002-sensors-actuators")
- **title**: string (main title of the module)
- **description**: string (brief description)
- **targetAudience**: array of strings (university students, AI/robotics learners, developers transitioning from software AI)
- **prerequisites**: array of strings (Module 1: ROS 2 fundamentals, basic Python programming)
- **learningObjectives**: array of strings (sensor understanding, actuator control, AI integration)
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
- **sensorType**: string (for sensor-focused chapters: IMU, LIDAR, CAMERA, etc.)
- **actuatorType**: string (for actuator-focused chapters: SERVO, MOTOR, etc.)
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
- **sensorSimulation**: boolean (whether tutorial involves sensor simulation)
- **actuatorControl**: boolean (whether tutorial involves actuator control)
- **aiIntegration**: boolean (whether tutorial involves AI decision-making)
- **createdAt**: datetime
- **updatedAt**: datetime
- **status**: enum (DRAFT | REVIEW | PUBLISHED)

## Sensor Entities

### Sensor
- **id**: string (unique identifier for documentation purposes)
- **name**: string (sensor name)
- **type**: enum (IMU | LIDAR | CAMERA | SONAR | GPS | OTHER)
- **description**: string (what the sensor measures)
- **measurementType**: string (e.g., "acceleration, angular velocity, orientation" for IMU)
- **dataRate**: float (measurements per second)
- **accuracy**: string (accuracy specifications)
- **rosMessageType**: string (ROS 2 message type, e.g., "sensor_msgs/msg/Imu")
- **createdAt**: datetime
- **updatedAt**: datetime

### IMUSensor
- **id**: string (inherits from Sensor)
- **linearAccelerationCovariance**: array of float (3x3 covariance matrix)
- **angularVelocityCovariance**: array of float (3x3 covariance matrix)
- **orientationCovariance**: array of float (3x3 covariance matrix)
- **frameId**: string (coordinate frame)

### LIDARSensor
- **id**: string (inherits from Sensor)
- **rangeMin**: float (minimum range in meters)
- **rangeMax**: float (maximum range in meters)
- **angleMin**: float (minimum angle in radians)
- **angleMax**: float (maximum angle in radians)
- **angleIncrement**: float (angle increment in radians)
- **scanTime**: float (time between scans in seconds)
- **timeIncrement**: float (time increment between measurements)

### CameraSensor
- **id**: string (inherits from Sensor)
- **imageWidth**: integer (image width in pixels)
- **imageHeight**: integer (image height in pixels)
- **fovHorizontal**: float (horizontal field of view in radians)
- **fovVertical**: float (vertical field of view in radians)
- **pixelFormat**: string (e.g., "rgb8", "bgr8", "mono8")

## Actuator Entities

### Actuator
- **id**: string (unique identifier for documentation purposes)
- **name**: string (actuator name)
- **type**: enum (SERVO | MOTOR | LINEAR_ACTUATOR | GRIPPER | OTHER)
- **description**: string (what the actuator does)
- **controlType**: enum (POSITION | VELOCITY | TORQUE | PWM)
- **rangeMin**: float (minimum control value)
- **rangeMax**: float (maximum control value)
- **maxSpeed**: float (maximum speed if applicable)
- **maxTorque**: float (maximum torque if applicable)
- **createdAt**: datetime
- **updatedAt**: datetime

### ServoActuator
- **id**: string (inherits from Actuator)
- **positionResolution**: float (smallest position change in radians)
- **gearRatio**: float (gear ratio if applicable)
- **controlSignalType**: string (e.g., "PWM", "digital", "analog")

### MotorActuator
- **id**: string (inherits from Actuator)
- **continuousRotation**: boolean (whether it can rotate continuously)
- **encoderResolution**: integer (encoder ticks per revolution)
- **gearRatio**: float (gear ratio if applicable)

## ROS 2 Integration Entities

### SensorDriver
- **id**: string (unique identifier for documentation purposes)
- **name**: string (driver name)
- **sensorType**: Sensor reference
- **supportedPlatforms**: array of string (platforms where driver works)
- **parameters**: array of ROS2Parameter objects
- **topicsPublished**: array of ROS2Topic objects
- **createdAt**: datetime
- **updatedAt**: datetime

### AIController
- **id**: string (unique identifier for documentation purposes)
- **name**: string (controller name)
- **inputTopics**: array of ROS2Topic references (sensor data inputs)
- **outputTopics**: array of ROS2Topic references (actuator command outputs)
- **algorithmType**: string (rule-based, ML, etc.)
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

### Sensor Validation
- Name must be unique within the module
- Type must be one of the defined enum values
- Data rate must be positive
- ROS message type must be valid

### Actuator Validation
- Name must be unique within the module
- Type must be one of the defined enum values
- Range min must be less than range max
- Control type must be appropriate for actuator type

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
- Module has many Sensor examples
- Module has many Actuator examples
- Tutorial may involve many Sensors
- Tutorial may involve many Actuators
- AIController receives data from many Sensors
- AIController controls many Actuators
- SensorDriver publishes to many ROS2Topic