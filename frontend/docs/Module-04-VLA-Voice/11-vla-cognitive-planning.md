
# VLA Cognitive Planning: LLMs to ROS 2 Actions

:::info Learning Objectives
* Understand how Large Language Models (LLMs) can enable cognitive planning for robots
* Implement LLM-based task decomposition for humanoid robots
* Integrate LLMs with ROS 2 action servers for complex behaviors
* Design prompt engineering strategies for robotic applications
:::

![Hero Diagram: LLMs to ROS 2 Actions](../../static/img/cognitive-planning.png)


Large Language Models (LLMs) represent a breakthrough in enabling cognitive planning for robots, allowing them to understand high-level goals and decompose them into executable actions. This cognitive layer bridges natural language commands with low-level robot control, enabling more intuitive human-robot interaction.

## Cognitive Planning Architecture

The cognitive planning system connects LLMs with ROS 2 through multiple layers:

- **Natural language interface**: Processing high-level commands
- **Task decomposition**: Breaking down complex goals into subtasks
- **Action mapping**: Converting subtasks to ROS 2 actions
- **Execution monitoring**: Tracking progress and handling failures

### LLM Integration Example

```python
import rclpy
from rclpy.node import Node
import openai
import json
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner')

        # LLM configuration
        openai.api_key = "your-api-key"

        # Subscribers and publishers
        self.goal_sub = self.create_subscription(
            String,
            'high_level_goal',
            self.goal_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            'planning_status',
            10
        )

    def goal_callback(self, msg):
        """Process high-level goal using LLM"""
        try:
            # Use LLM to decompose the goal
            plan = self.generate_plan(msg.data)

            # Execute the plan
            self.execute_plan(plan)

        except Exception as e:
            self.get_logger().error(f'Planning error: {e}')

    def generate_plan(self, goal_text):
        """Generate executable plan using LLM"""
        prompt = f"""
        You are a robot task planner. Decompose the following goal into specific, executable steps.
        Respond in JSON format with the following structure:

        {{
            "steps": [
                {{
                    "action": "action_name",
                    "parameters": {{"param1": "value1", "param2": "value2"}},
                    "description": "Human-readable description"
                }}
            ]
        }}

        Goal: {goal_text}
        """

        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1
        )

        plan_json = response.choices[0].message.content
        return json.loads(plan_json)

    def execute_plan(self, plan):
        """Execute the generated plan"""
        for step in plan['steps']:
            self.execute_step(step)

def main(args=None):
    rclpy.init(args=args)
    planner = CognitivePlannerNode()
    rclpy.spin(planner)
    rclpy.shutdown()
```

:::tip Key Concept
Cognitive planning with LLMs requires careful prompt engineering to ensure the generated plans are executable by the robot and account for environmental constraints and safety requirements.
:::

## Task Decomposition Strategies

LLMs can decompose complex tasks into manageable subtasks:

```python
class TaskDecomposer:
    def __init__(self):
        self.knowledge_base = {
            'navigation': ['move_base', 'path_planning', 'obstacle_avoidance'],
            'manipulation': ['arm_control', 'grasp_planning', 'object_manipulation'],
            'interaction': ['speech_synthesis', 'gesture_control', 'facial_expression']
        }

    def decompose_task(self, goal, context):
        """Decompose high-level goal with environmental context"""
        prompt = f"""
        Decompose this goal into specific robot actions considering the context:

        Goal: {goal}
        Context: {context}

        Available capabilities: {list(self.knowledge_base.keys())}

        Return JSON with executable steps.
        """

        # Implementation would call LLM with this prompt
        pass
```

## ROS 2 Action Integration

The cognitive planner must interface with ROS 2 action servers:

```python
from rclpy.action import ActionClient
from move_base_msgs.action import MoveBase
from humanoid_msgs.action import ManipulateObject

class ActionExecutor:
    def __init__(self, node):
        self.node = node

        # Action clients
        self.move_base_client = ActionClient(node, MoveBase, 'move_base')
        self.manipulate_client = ActionClient(node, ManipulateObject, 'manipulate_object')

    async def execute_navigation(self, target_pose):
        """Execute navigation action"""
        goal_msg = MoveBase.Goal()
        goal_msg.target_pose = target_pose

        self.node.get_logger().info('Sending navigation goal...')
        future = self.move_base_client.send_goal_async(goal_msg)
        result = await future
        return result.result

    async def execute_manipulation(self, object_name, action_type):
        """Execute manipulation action"""
        goal_msg = ManipulateObject.Goal()
        goal_msg.object_name = object_name
        goal_msg.action_type = action_type

        future = self.manipulate_client.send_goal_async(goal_msg)
        result = await future
        return result.result
```

## Prompt Engineering for Robotics

Effective prompt engineering is crucial for reliable cognitive planning:

- **Context provision**: Include robot capabilities and environmental constraints
- **Structured output**: Request specific JSON formats for easy parsing
- **Safety constraints**: Include safety and ethical guidelines
- **Error handling**: Plan for ambiguous or impossible requests

### Example Prompt Template

```python
def create_planning_prompt(goal, robot_capabilities, environment, safety_constraints):
    return f"""
    You are a cognitive planner for a humanoid robot. Plan the following task:

    GOAL: {goal}

    ROBOT CAPABILITIES: {robot_capabilities}
    ENVIRONMENT: {environment}
    SAFETY CONSTRAINTS: {safety_constraints}

    INSTRUCTIONS:
    1. Decompose the goal into executable steps
    2. Ensure each step is achievable with the robot's capabilities
    3. Respect all safety constraints
    4. Output in the following JSON format:

    {{
        "plan": {{
            "goal": "{goal}",
            "steps": [
                {{
                    "id": 1,
                    "action": "action_name",
                    "parameters": {{"param1": "value1"}},
                    "preconditions": ["condition1", "condition2"],
                    "effects": ["effect1", "effect2"]
                }}
            ]
        }}
    }}

    Be precise and ensure the plan is executable.
    """
```

## Hands-on Lab

Implement a cognitive planning system with:
1. LLM integration for task decomposition
2. ROS 2 action server interface
3. Prompt engineering for reliable output
4. Plan execution and monitoring
5. Error handling and plan adjustment

## Self-Assessment

1. How do LLMs enable cognitive planning in robotics?
2. What are the challenges of integrating LLMs with ROS 2?
3. Why is prompt engineering important for robotic applications?