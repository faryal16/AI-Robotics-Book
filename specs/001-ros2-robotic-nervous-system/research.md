# Research: Module 1: The Robotic Nervous System (ROS 2)

## Decision: ROS 2 Distribution Choice
**Rationale**: Using ROS 2 Humble Hawksbill (LTS) as it provides long-term support, extensive documentation, and is the current standard for industrial and educational applications.

**Alternatives considered**:
- ROS 2 Galactic: Short-term support, less stable
- ROS 2 Rolling: Bleeding edge but less stable for educational content
- ROS 1: Legacy, no longer supported for new projects

## Decision: Python vs C++ Focus
**Rationale**: Focusing on Python (rclpy) as it's more accessible to the target audience (AI developers transitioning to robotics) and allows for faster prototyping and learning.

**Alternatives considered**:
- C++ (rclcpp): More performant but steeper learning curve
- Both languages: Would make content too complex for beginners
- Other languages: Less common in ROS 2 ecosystem

## Decision: URDF as Robot Description Format
**Rationale**: URDF is the standard robot description format in ROS ecosystem, well-documented, and essential for any ROS-based robotics work.

**Alternatives considered**:
- SDF (Simulation Description Format): More simulation-focused
- Custom formats: Would not be compatible with ROS tools
- XACRO: More advanced, can be introduced later as extension to URDF

## Decision: Hands-on Lab Approach
**Rationale**: Practical implementation is essential for learning ROS 2 concepts. The humanoid URDF creation and joint control lab directly addresses the success criteria.

**Alternatives considered**:
- Simulation only: Less engaging for learners
- Theory only: Would not meet hands-on requirements
- Pre-built examples only: Less learning value than building from scratch

## Decision: Content Structure by User Stories
**Rationale**: Organizing content around the three user stories (P1, P2, P3) ensures comprehensive coverage from basic concepts to advanced applications.

**Alternatives considered**:
- Topic-based organization: Might not follow logical learning progression
- Chronological approach: Less focused on learning outcomes
- Project-based: Could be too complex for beginners