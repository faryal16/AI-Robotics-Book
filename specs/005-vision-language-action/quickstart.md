# Quickstart: Module 5: Vision-Language-Action

## Prerequisites

- Completed Module 1-4 (Complete series)
- Ubuntu 22.04 LTS with ROS 2 Humble installed
- Python 3.10+
- Git for version control
- OpenAI API key (for LLM functionality) or local LLM setup
- Microphone for voice input (for voice processing examples)

## Setup Multimodal Environment

### 1. Install Core Dependencies
```bash
pip3 install openai torch torchvision torchaudio transformers
pip3 install datasets evaluate
pip3 install openai-whisper
pip3 install opencv-python numpy pillow
pip3 install speechrecognition pyaudio
```

### 2. Install ROS 2 Dependencies
```bash
sudo apt update
sudo apt install ros-humble-vision-opencv ros-humble-cv-bridge
sudo apt install ros-humble-control-msgs ros-humble-action-msgs
sudo apt install python3-pyaudio python3-speechRecognition
```

### 3. Set OpenAI API Key (if using OpenAI services)
```bash
export OPENAI_API_KEY="your-api-key-here"
```

## Voice Processing and Whisper Integration

### 1. Create a Voice Processing Package
```bash
mkdir -p ~/ros2_ws/src/vla_examples
cd ~/ros2_ws/src/vla_examples
ros2 pkg create --build-type ament_python voice_language_action --dependencies rclpy std_msgs sensor_msgs geometry_msgs cv_bridge action_msgs
```

### 2. Voice-to-Action Node
Create `~/ros2_ws/src/vla_examples/voice_language_action/voice_to_action.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import openai
import speech_recognition as sr
import threading
import queue
import time

class VoiceToActionNode(Node):
    def __init__(self):
        super().__init__('voice_to_action_node')

        # Publisher for recognized commands
        self.command_pub = self.create_publisher(String, '/voice/command', 10)

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Set energy threshold for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Start voice recognition in a separate thread
        self.voice_queue = queue.Queue()
        self.voice_thread = threading.Thread(target=self.voice_recognition_loop)
        self.voice_thread.daemon = True
        self.voice_thread.start()

        # Timer to process recognized commands
        self.timer = self.create_timer(0.1, self.process_voice_commands)

        self.get_logger().info('Voice-to-Action node initialized')

    def voice_recognition_loop(self):
        """Continuously listen for voice commands"""
        while rclpy.ok():
            try:
                with self.microphone as source:
                    self.get_logger().info("Listening for voice command...")
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=10)

                # Use Whisper for speech recognition
                try:
                    # For online Whisper API (requires OpenAI API key)
                    text = self.recognizer.recognize_whisper_api(audio, model="whisper-1")
                    self.voice_queue.put(text)
                    self.get_logger().info(f"Recognized: {text}")
                except sr.RequestError:
                    # Fallback: use offline recognition
                    try:
                        text = self.recognizer.recognize_google(audio)
                        self.voice_queue.put(text)
                        self.get_logger().info(f"Recognized (Google): {text}")
                    except sr.UnknownValueError:
                        self.get_logger().info("Could not understand audio")
                    except sr.RequestError as e:
                        self.get_logger().error(f"Could not request results from Google; {e}")

            except sr.WaitTimeoutError:
                # No speech detected, continue listening
                continue
            except Exception as e:
                self.get_logger().error(f"Error in voice recognition: {e}")
                time.sleep(1)  # Brief pause before retrying

    def process_voice_commands(self):
        """Process recognized voice commands"""
        try:
            while True:  # Process all items in queue
                command_text = self.voice_queue.get_nowait()

                # Publish the recognized command
                cmd_msg = String()
                cmd_msg.data = command_text
                self.command_pub.publish(cmd_msg)

                self.get_logger().info(f'Published voice command: "{command_text}"')
        except queue.Empty:
            pass  # No commands in queue, continue

def main(args=None):
    rclpy.init(args=args)
    voice_to_action_node = VoiceToActionNode()

    try:
        rclpy.spin(voice_to_action_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_to_action_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. LLM Cognitive Planning Node
Create `~/ros2_ws/src/vla_examples/voice_language_action/llm_planner.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
import openai
import json
import time

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')

        # Subscriber for voice commands
        self.command_sub = self.create_subscription(
            String,
            '/voice/command',
            self.command_callback,
            10
        )

        # Publisher for planned actions
        self.action_pub = self.create_publisher(String, '/planned/action_sequence', 10)

        # Initialize OpenAI client
        try:
            self.openai_client = openai.OpenAI()
        except Exception as e:
            self.get_logger().error(f"Could not initialize OpenAI client: {e}")
            self.openai_client = None

        self.get_logger().info('LLM Planner node initialized')

    def command_callback(self, msg):
        """Process incoming voice commands and generate action plans"""
        command_text = msg.data
        self.get_logger().info(f'Received command: "{command_text}"')

        # Generate action plan using LLM
        action_plan = self.generate_action_plan(command_text)

        if action_plan:
            # Publish the action plan
            plan_msg = String()
            plan_msg.data = json.dumps(action_plan)
            self.action_pub.publish(plan_msg)
            self.get_logger().info(f'Published action plan: {action_plan}')

    def generate_action_plan(self, command_text):
        """Use LLM to generate a sequence of actions for the given command"""
        if not self.openai_client:
            self.get_logger().error("OpenAI client not initialized")
            return None

        try:
            # Define the system prompt to guide the LLM
            system_prompt = """
            You are a cognitive planner for a humanoid robot. Your job is to convert high-level natural language commands into sequences of specific actions that the robot can execute using ROS 2.

            The robot has the following capabilities:
            1. Navigation: Move to specific locations
            2. Object Recognition: Detect and identify objects in the environment
            3. Manipulation: Pick up, move, and place objects
            4. Interaction: Open doors, press buttons, etc.

            Please convert the user's command into a sequence of specific actions with parameters. Return the result as a JSON object with the following structure:
            {
                "task_description": "Brief description of the task",
                "action_sequence": [
                    {
                        "action_type": "navigation|perception|manipulation|interaction",
                        "description": "What the action does",
                        "parameters": {
                            "location": "x, y coordinates for navigation",
                            "object": "name of object for perception/manipulation",
                            "pose": "position and orientation for manipulation"
                        }
                    }
                ]
            }

            Keep the actions simple and executable. Focus on achievable tasks given the robot's capabilities.
            """

            # Create the prompt for the LLM
            user_prompt = f"Command: {command_text}"

            # Call the OpenAI API
            response = self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
                max_tokens=500
            )

            # Extract the response
            plan_text = response.choices[0].message.content.strip()

            # Clean up the response to extract JSON
            if plan_text.startswith("```json"):
                plan_text = plan_text[7:]  # Remove ```json
            if plan_text.endswith("```"):
                plan_text = plan_text[:-3]  # Remove ```

            # Parse the JSON response
            action_plan = json.loads(plan_text)
            return action_plan

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error parsing LLM response as JSON: {e}")
            self.get_logger().info(f"LLM response: {plan_text}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error calling LLM: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    llm_planner_node = LLMPlannerNode()

    try:
        rclpy.spin(llm_planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        llm_planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. Vision Processing Node
Create `~/ros2_ws/src/vla_examples/voice_language_action/vision_processor.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
import json

class VisionProcessorNode(Node):
    def __init__(self):
        super().__init__('vision_processor_node')

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for detected objects
        self.detection_pub = self.create_publisher(String, '/vision/detections', 10)

        # For demonstration, we'll use simple color-based detection
        # In a real implementation, you would use a deep learning model
        self.get_logger().info('Vision Processor node initialized')

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform simple object detection (for demonstration)
            # In practice, you would use a trained model like YOLO
            detections = self.simple_object_detection(cv_image)

            # Publish detections
            if detections:
                detection_msg = String()
                detection_msg.data = json.dumps(detections)
                self.detection_pub.publish(detection_msg)
                self.get_logger().info(f'Published detections: {detections}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def simple_object_detection(self, image):
        """Simple color-based object detection for demonstration"""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red color (for demonstration)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        # Upper red range (for red that wraps around in HSV)
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        # Combine masks
        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Filter out small areas
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)

                detection = {
                    "object_class": "red_object",  # For demonstration
                    "confidence": 0.8,  # For demonstration
                    "bbox": [int(x), int(y), int(w), int(h)],
                    "center": [int(x + w/2), int(y + h/2)]
                }
                detections.append(detection)

        return detections

def main(args=None):
    rclpy.init(args=args)
    vision_processor_node = VisionProcessorNode()

    try:
        rclpy.spin(vision_processor_node)
    except KeyboardInterrupt:
        pass
    finally:
        vision_processor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Action Execution Node

### 1. Create Action Execution Node
Create `~/ros2_ws/src/vla_examples/voice_language_action/action_executor.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatus
import json
import time

class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__('action_executor_node')

        # Subscriber for action plans
        self.plan_sub = self.create_subscription(
            String,
            '/planned/action_sequence',
            self.plan_callback,
            10
        )

        # Publishers for different action types
        self.nav_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.manip_pub = self.create_publisher(String, '/manipulation/command', 10)

        self.get_logger().info('Action Executor node initialized')

    def plan_callback(self, msg):
        """Execute the action plan received from the LLM planner"""
        try:
            plan = json.loads(msg.data)
            self.get_logger().info(f'Executing plan: {plan["task_description"]}')

            # Execute each action in sequence
            for i, action in enumerate(plan['action_sequence']):
                self.get_logger().info(f'Executing action {i+1}: {action["description"]}')

                success = self.execute_action(action)

                if not success:
                    self.get_logger().error(f'Action {i+1} failed: {action["description"]}')
                    break
                else:
                    self.get_logger().info(f'Action {i+1} completed successfully')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing action plan: {e}')
        except Exception as e:
            self.get_logger().error(f'Error executing plan: {e}')

    def execute_action(self, action):
        """Execute a single action based on its type"""
        action_type = action['action_type']
        params = action.get('parameters', {})

        if action_type == 'navigation':
            return self.execute_navigation(params)
        elif action_type == 'perception':
            return self.execute_perception(params)
        elif action_type == 'manipulation':
            return self.execute_manipulation(params)
        elif action_type == 'interaction':
            return self.execute_interaction(params)
        else:
            self.get_logger().error(f'Unknown action type: {action_type}')
            return False

    def execute_navigation(self, params):
        """Execute navigation action"""
        try:
            location = params.get('location')
            if location:
                x, y = location[0], location[1]

                # Create navigation goal
                goal = PoseStamped()
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.header.frame_id = 'map'
                goal.pose.position.x = float(x)
                goal.pose.position.y = float(y)
                goal.pose.position.z = 0.0
                goal.pose.orientation.w = 1.0  # No rotation

                # Publish navigation goal
                self.nav_pub.publish(goal)
                self.get_logger().info(f'Published navigation goal to ({x}, {y})')

                # In a real implementation, you would wait for navigation completion
                time.sleep(2)  # Simulate navigation time
                return True
            else:
                self.get_logger().error('Navigation action missing location parameter')
                return False
        except Exception as e:
            self.get_logger().error(f'Error in navigation: {e}')
            return False

    def execute_perception(self, params):
        """Execute perception action"""
        try:
            obj_name = params.get('object')
            self.get_logger().info(f'Looking for object: {obj_name}')

            # In a real implementation, you would trigger vision processing
            # and wait for object detection results
            time.sleep(1)  # Simulate perception time
            return True
        except Exception as e:
            self.get_logger().error(f'Error in perception: {e}')
            return False

    def execute_manipulation(self, params):
        """Execute manipulation action"""
        try:
            obj_name = params.get('object')
            pose = params.get('pose')

            self.get_logger().info(f'Manipulating object: {obj_name}')
            if pose:
                self.get_logger().info(f'At pose: {pose}')

            # In a real implementation, you would control robot arms/actuators
            time.sleep(2)  # Simulate manipulation time
            return True
        except Exception as e:
            self.get_logger().error(f'Error in manipulation: {e}')
            return False

    def execute_interaction(self, params):
        """Execute interaction action"""
        try:
            interaction_type = params.get('type')
            self.get_logger().info(f'Performing interaction: {interaction_type}')

            # In a real implementation, you would interact with environment
            time.sleep(1)  # Simulate interaction time
            return True
        except Exception as e:
            self.get_logger().error(f'Error in interaction: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    action_executor_node = ActionExecutorNode()

    try:
        rclpy.spin(action_executor_node)
    except KeyboardInterrupt:
        pass
    finally:
        action_executor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5. Update setup.py
Update `~/ros2_ws/src/vla_examples/setup.py` to include entry points:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'voice_language_action'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Examples for vision-language-action integration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_to_action = voice_language_action.voice_to_action:main',
            'llm_planner = voice_language_action.llm_planner:main',
            'vision_processor = voice_language_action.vision_processor:main',
            'action_executor = voice_language_action.action_executor:main',
        ],
    },
)
```

## Running the Complete Vision-Language-Action Pipeline

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select voice_language_action
source ~/ros2_ws/install/setup.bash
```

### 2. Run the Complete Pipeline
```bash
# Terminal 1: Start the voice recognition node
ros2 run voice_language_action voice_to_action

# Terminal 2: Start the LLM planner
ros2 run voice_language_action llm_planner

# Terminal 3: Start the vision processor (you'll need a camera or simulated camera)
ros2 run voice_language_action vision_processor

# Terminal 4: Start the action executor
ros2 run voice_language_action action_executor
```

### 3. Alternative: Run with a Camera Simulator
If you don't have a real camera, you can simulate one:

```bash
# Install image publisher if not already installed
sudo apt install ros-humble-vision-opencv

# Publish a test image
ros2 run image_publisher image_publisher_node --ros-args -p image_path:=/path/to/test/image.jpg
```

## Testing Your Setup

Verify your VLA setup:
```bash
# Check if all nodes are available
ros2 run voice_language_action --help

# Check available topics
ros2 topic list

# Listen to voice commands (in a separate terminal)
ros2 topic echo /voice/command

# Listen to action plans (in a separate terminal)
ros2 topic echo /planned/action_sequence
```

## Running the Module's Hands-on Lab

The hands-on lab for this module involves creating a complete vision-language-action pipeline that can:
1. Accept voice commands using Whisper
2. Process commands with an LLM to generate action plans
3. Use vision to perceive the environment
4. Execute actions based on the plan

For the lab, you'll integrate all the components created in the examples above into a cohesive system that can respond to voice commands with appropriate actions.

## Troubleshooting

- If OpenAI API gives authentication errors: Verify your API key is set correctly
- If voice recognition doesn't work: Check microphone permissions and PyAudio installation
- If ROS 2 nodes can't communicate: Verify all nodes are on the same ROS domain
- If vision processing is slow: Consider using a lighter model or optimizing image processing