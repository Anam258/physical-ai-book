![Hero Diagram: Whisper & Voice Commands](../../static/img/VLA-voice-action.png)

:::info Learning Objectives
* Implement voice command recognition using Whisper and similar models
* Integrate speech-to-text capabilities with ROS 2 systems
* Design voice command grammars for humanoid robot control
* Process natural language commands for robot action execution
:::

# VLA Voice Action: Whisper & Voice Commands

Voice-based interaction represents a natural and intuitive way for humans to communicate with humanoid robots. Leveraging models like OpenAI's Whisper for speech recognition, robots can understand and execute complex voice commands, enabling seamless human-robot interaction in various environments.

## Voice Command Architecture

The voice command system for humanoid robots typically involves:

- **Speech recognition**: Converting audio to text using models like Whisper
- **Natural language understanding**: Parsing text commands into executable actions
- **Command mapping**: Translating understood commands to ROS 2 actions
- **Response generation**: Providing feedback to the user

### Whisper Integration Example

```python
import rclpy
from rclpy.node import Node
import whisper
import pyaudio
import wave
import threading
from std_msgs.msg import String

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Load Whisper model
        self.whisper_model = whisper.load_model("base")

        # Audio recording parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.record_seconds = 5

        # Publishers and subscribers
        self.command_pub = self.create_publisher(String, 'voice_command', 10)

        # Start audio recording thread
        self.recording = True
        self.audio_thread = threading.Thread(target=self.record_audio)
        self.audio_thread.start()

    def record_audio(self):
        """Record audio and process with Whisper"""
        p = pyaudio.PyAudio()

        stream = p.open(format=self.format,
                        channels=self.channels,
                        rate=self.rate,
                        input=True,
                        frames_per_buffer=self.chunk)

        while self.recording:
            frames = []

            # Record audio
            for i in range(0, int(self.rate / self.chunk * self.record_seconds)):
                data = stream.read(self.chunk)
                frames.append(data)

            # Save to temporary file
            wf = wave.open("temp_audio.wav", 'wb')
            wf.setnchannels(self.channels)
            wf.setsampwidth(p.get_sample_size(self.format))
            wf.setframerate(self.rate)
            wf.writeframes(b''.join(frames))
            wf.close()

            # Process with Whisper
            result = self.whisper_model.transcribe("temp_audio.wav")
            text = result["text"]

            # Process the recognized text
            self.process_command(text)

    def process_command(self, text):
        """Process recognized text and publish command"""
        cmd_msg = String()
        cmd_msg.data = text
        self.command_pub.publish(cmd_msg)
        self.get_logger().info(f'Recognized: {text}')

def main(args=None):
    rclpy.init(args=args)
    voice_node = VoiceCommandNode()

    try:
        rclpy.spin(voice_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_node.recording = False
        voice_node.audio_thread.join()
        rclpy.shutdown()
```

:::tip Key Concept
Voice command systems for humanoid robots must handle ambient noise, multiple speakers, and real-time processing requirements while maintaining accuracy in diverse acoustic environments.
:::

## Natural Language Command Processing

Humanoid robots need to interpret natural language commands that may vary in structure and vocabulary:

```python
class CommandInterpreter:
    def __init__(self):
        self.command_patterns = {
            'move_forward': [
                'move forward',
                'go forward',
                'walk forward',
                'step forward'
            ],
            'turn_left': [
                'turn left',
                'rotate left',
                'pivot left'
            ],
            'greet': [
                'say hello',
                'wave hello',
                'greet me',
                'hello'
            ]
        }

    def interpret_command(self, text):
        """Interpret natural language command"""
        text_lower = text.lower().strip()

        for action, patterns in self.command_patterns.items():
            for pattern in patterns:
                if pattern in text_lower:
                    return action, self.extract_parameters(text_lower, pattern)

        return 'unknown', {}

    def extract_parameters(self, text, pattern):
        """Extract parameters from command"""
        # Example: extract distance from "move forward 2 meters"
        import re
        match = re.search(r'(\d+(?:\.\d+)?)\s*(meter|metre|m)', text)
        if match:
            return {'distance': float(match.group(1))}
        return {}
```

## ROS 2 Integration

Voice commands must be integrated with ROS 2 action servers and services:

```python
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from humanoid_msgs.action import MoveBase

class VoiceController(Node):
    def __init__(self):
        super().__init__('voice_controller')

        # Command subscription
        self.command_sub = self.create_subscription(
            String,
            'voice_command',
            self.command_callback,
            10
        )

        # Action clients for robot actions
        self.move_base_client = ActionClient(self, MoveBase, 'move_base')

        # Command interpreter
        self.interpreter = CommandInterpreter()

    def command_callback(self, msg):
        """Handle voice command"""
        command, params = self.interpreter.interpret_command(msg.data)

        if command == 'move_forward':
            self.execute_move_forward(params.get('distance', 1.0))
        elif command == 'turn_left':
            self.execute_turn_left()
        # ... handle other commands
```

## Real-time Performance Considerations

Voice command systems must balance accuracy with real-time performance:

- **Audio buffering**: Efficient audio capture and processing
- **Model optimization**: Using smaller models or quantized versions
- **Asynchronous processing**: Non-blocking command recognition
- **Hotword detection**: Wake word systems to activate listening

## Hands-on Lab

Create a voice command system with:
1. Whisper model integration for speech recognition
2. Natural language command interpretation
3. ROS 2 action execution based on voice commands
4. Audio input handling and processing
5. Real-time performance optimization

## Self-Assessment

1. How does Whisper differ from traditional speech recognition systems?
2. What are the challenges of voice command processing for robots?
3. How can natural language commands be mapped to robot actions?