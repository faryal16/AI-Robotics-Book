# Data Model: Module 1: The Robotic Nervous System (ROS 2)

## Educational Content Entities

### Module
- **id**: string (unique identifier: "001-ros2-robotic-nervous-system")
- **title**: string (main title of the module)
- **description**: string (brief description)
- **targetAudience**: array of strings (university students, AI/robotics learners, developers transitioning from software AI)
- **prerequisites**: array of strings (basic programming knowledge, Python familiarity)
- **learningObjectives**: array of strings (ROS 2 fundamentals, rclpy programming, URDF understanding)
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
- **createdAt**: datetime
- **updatedAt**: datetime
- **status**: enum (DRAFT | REVIEW | PUBLISHED)

## ROS 2 Entities

### ROS2Node
- **id**: string (unique identifier for documentation purposes)
- **name**: string (node name)
- **description**: string (what the node does)
- **nodeType**: enum (PUBLISHER | SUBSCRIBER | SERVICE_SERVER | SERVICE_CLIENT | ACTION_SERVER | ACTION_CLIENT)
- **topicsPublished**: array of ROS2Topic references
- **topicsSubscribed**: array of ROS2Topic references
- **servicesOffered**: array of ROS2Service references
- **servicesUsed**: array of ROS2Service references
- **parameters**: array of ROS2Parameter objects
- **codeExample**: string (Python code example using rclpy)

### ROS2Topic
- **id**: string (unique identifier for documentation purposes)
- **name**: string (topic name with namespace)
- **type**: string (message type, e.g., std_msgs/msg/String)
- **description**: string (what data is published/subscribed)
- **publishers**: array of ROS2Node references
- **subscribers**: array of ROS2Node references
- **qosProfile**: string (quality of service settings)

### ROS2Service
- **id**: string (unique identifier for documentation purposes)
- **name**: string (service name with namespace)
- **type**: string (service type, e.g., std_srvs/srv/SetBool)
- **description**: string (what the service does)
- **server**: ROS2Node reference
- **clients**: array of ROS2Node references

### ROS2Parameter
- **id**: string (unique identifier for documentation purposes)
- **name**: string (parameter name)
- **type**: string (parameter type, e.g., string, int, double)
- **description**: string (what the parameter controls)
- **defaultValue**: string (default value)
- **constraints**: string (min/max values, allowed values)

## Robot Model Entities

### URDFModel
- **id**: string (unique identifier for documentation purposes)
- **name**: string (model name)
- **description**: string (what the model represents)
- **links**: array of URDFLink objects
- **joints**: array of URDFJoint objects
- **materials**: array of URDFMaterial objects
- **gazeboExtensions**: object (Gazebo-specific extensions)
- **filePath**: string (path to URDF file)

### URDFLink
- **id**: string (unique identifier for documentation purposes)
- **name**: string (link name)
- **visual**: URDFVisual object
- **collision**: URDFVisual object
- **inertial**: URDFInertial object
- **parentJoint**: string (name of parent joint, if any)

### URDFJoint
- **id**: string (unique identifier for documentation purposes)
- **name**: string (joint name)
- **type**: enum (revolute | continuous | prismatic | fixed | floating | planar)
- **parent**: string (parent link name)
- **child**: string (child link name)
- **origin**: URDFOrigin object
- **axis**: array of float (x, y, z axis of rotation/translation)
- **limits**: URDFJointLimits object (if applicable)

### URDFJointLimits
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

### ROS2Node Validation
- Name must follow ROS naming conventions
- Type must be one of the defined enum values
- Code example must be valid Python syntax

### URDFModel Validation
- Name must be unique within the module
- Must have at least one link
- Joint parent/child relationships must be valid
- URDF syntax must be valid XML

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
- Module has many ROS2Node examples
- Module has many URDFModel examples
- ROS2Node publishes/subscribes to ROS2Topic
- ROS2Node offers/uses ROS2Service
- URDFModel contains many URDFLink and URDFJoint